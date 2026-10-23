import dataclasses
import math
import os
import re
import select
import socket
import stat
import threading
import time
from typing import Iterable


MAX_DATAGRAM_SIZE = 4096
_IDENTIFIER_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
_RECORD_TYPES = frozenset(("OBJECT", "SURFACE"))
_SIDES = frozenset(("left", "right"))


class ProtocolError(ValueError):
    pass


@dataclasses.dataclass(frozen=True)
class StateRecord:
    record_type: str
    object_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    size: tuple[float, float, float]
    acquisition_time_ns: int | None = None


def _validate_identifier(value: str) -> str:
    if not isinstance(value, str) or _IDENTIFIER_RE.fullmatch(value) is None:
        raise ProtocolError(f"invalid identifier: {value!r}")
    return value


def parse_state_record(datagram: bytes) -> StateRecord:
    """Parse one strict simulator object-state datagram."""
    if not isinstance(datagram, bytes):
        raise ProtocolError("state record must be bytes")
    if len(datagram) > MAX_DATAGRAM_SIZE:
        raise ProtocolError("state datagram is oversized")
    try:
        text = datagram.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("state record is not ASCII") from exc

    fields = text.split(" ")
    versioned = fields[0] in ('OBJECT_V2','SURFACE_V2')
    if len(fields) != (13 if versioned else 12) or any(not field for field in fields):
        raise ProtocolError("state record must contain exactly 12 legacy or 13 V2 space-separated fields")
    record_type, object_id = fields[:2]
    acquisition_time_ns = None
    if versioned:
        record_type = record_type[:-3]
        if not fields[-1].isdigit() or not 0<int(fields[-1])<2**63:
            raise ProtocolError('invalid acquisition timestamp')
        acquisition_time_ns = int(fields[-1])
    if record_type not in _RECORD_TYPES:
        raise ProtocolError(f"unknown state record type: {record_type!r}")
    _validate_identifier(object_id)
    try:
        values = tuple(float(field) for field in fields[2:12])
    except ValueError as exc:
        raise ProtocolError("state record contains a non-numeric value") from exc
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("state record contains a nonfinite value")

    qx, qy, qz, qw = values[3:7]
    quaternion_norm = math.hypot(qx, qy, qz, qw)
    if quaternion_norm == 0.0:
        raise ProtocolError("state record contains an invalid quaternion")
    size = values[7:10]
    if any(dimension <= 0.0 for dimension in size):
        raise ProtocolError("state record dimensions must be positive")

    return StateRecord(
        record_type=record_type,
        object_id=object_id,
        position=values[0:3],
        orientation=tuple(component / quaternion_norm for component in (qx, qy, qz, qw)),
        size=size,
        acquisition_time_ns=acquisition_time_ns,
    )


class StateCache:
    """Latest records with receipt expiry and V2 acquisition replay rejection."""

    def __init__(self) -> None:
        self._records: dict[tuple[str, str], tuple[float, StateRecord]] = {}

    def update(self, record: StateRecord, received_at: float) -> None:
        if not math.isfinite(received_at):
            raise ValueError("receipt timestamp must be finite")
        key = (record.record_type,record.object_id)
        previous = self._records.get(key)
        if previous is not None and previous[1].acquisition_time_ns is not None:
            if record.acquisition_time_ns is None or record.acquisition_time_ns<=previous[1].acquisition_time_ns:
                return
        self._records[key] = (received_at, record)

    def fresh_records(self, now: float, stale_timeout: float) -> tuple[StateRecord, ...]:
        if not math.isfinite(now) or not math.isfinite(stale_timeout) or stale_timeout < 0.0:
            raise ValueError("cache time and stale timeout must be finite and nonnegative")
        return tuple(
            record
            for received_at, record in self._records.values()
            if 0.0 <= now - received_at <= stale_timeout
        )


def format_manipulation_command(action: str, side: str, object_id: str | None = None) -> bytes:
    if side not in _SIDES:
        raise ProtocolError(f"invalid manipulation side: {side!r}")
    if action == "GRASP":
        if object_id is None:
            raise ProtocolError("GRASP requires an object identifier")
        return f"GRASP {side} {_validate_identifier(object_id)}".encode("ascii")
    if action == "RELEASE":
        if object_id is not None:
            raise ProtocolError("RELEASE does not accept an object identifier")
        return f"RELEASE {side}".encode("ascii")
    raise ProtocolError(f"invalid manipulation action: {action!r}")


def format_base_pin_command(pin: bool) -> bytes:
    if not isinstance(pin, bool):
        raise ProtocolError("base pin state must be boolean")
    return b"PIN_BASE" if pin else b"UNPIN_BASE"


def parse_ack(datagram: bytes) -> tuple[bool, str]:
    """Parse one strict simulator command acknowledgement."""
    if not isinstance(datagram, bytes):
        raise ProtocolError("command acknowledgement must be bytes")
    if len(datagram) > MAX_DATAGRAM_SIZE:
        raise ProtocolError("command acknowledgement is oversized")
    try:
        text = datagram.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("command acknowledgement is not ASCII") from exc
    if any(character < " " or character > "~" for character in text):
        raise ProtocolError("command acknowledgement contains non-printable ASCII")

    status, separator, message = text.partition(" ")
    if separator and message and message == message.strip(" "):
        if status == "OK":
            return True, message
        if status == "ERROR":
            return False, message
    raise ProtocolError("unexpected command acknowledgement")


def _remove_socket_path(path: str) -> None:
    try:
        mode = os.stat(path).st_mode
    except FileNotFoundError:
        return
    if not stat.S_ISSOCK(mode):
        raise RuntimeError(f"refusing to remove non-socket path: {path}")
    os.unlink(path)


class CommandClient:
    """Serialized request/acknowledgement client over Unix datagrams."""

    def __init__(self, server_path: str, response_path: str, ack_timeout: float) -> None:
        if not server_path or not response_path:
            raise ValueError("command socket paths must not be empty")
        if not math.isfinite(ack_timeout) or ack_timeout <= 0.0:
            raise ValueError("command acknowledgement timeout must be positive and finite")
        self._server_path = server_path
        self._response_path = response_path
        self._ack_timeout = ack_timeout
        self._lock = threading.Lock()
        self._socket = None

        _remove_socket_path(response_path)
        command_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            command_socket.setblocking(False)
            command_socket.bind(response_path)
        except Exception:
            command_socket.close()
            _remove_socket_path(response_path)
            raise
        self._socket = command_socket

    def send(self, command: bytes) -> tuple[bool, str]:
        with self._lock:
            if self._socket is None:
                return False, "command response socket is closed"

            try:
                while True:
                    try:
                        self._socket.recvfrom(MAX_DATAGRAM_SIZE + 1)
                    except BlockingIOError:
                        break

                sent = self._socket.sendto(command, self._server_path)
                if sent != len(command):
                    raise OSError(f"sent {sent} of {len(command)} bytes")
                readable, _, _ = select.select([self._socket], [], [], self._ack_timeout)
                if not readable:
                    return False, "command acknowledgement timed out"
                acknowledgement, sender = self._socket.recvfrom(MAX_DATAGRAM_SIZE + 1)
            except OSError as exc:
                return False, f"command socket failed: {exc}"

            if sender != self._server_path:
                return False, f"command acknowledgement came from unexpected socket: {sender}"
            try:
                return parse_ack(acknowledgement)
            except ProtocolError as exc:
                return False, str(exc)

    def close(self) -> None:
        with self._lock:
            if self._socket is not None:
                self._socket.close()
                self._socket = None
            _remove_socket_path(self._response_path)


def _create_node_class():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from std_srvs.srv import Trigger
    from g1_msgs.srv import GraspObject
    from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

    class SimManipulationIO(Node):
        def __init__(self) -> None:
            super().__init__("g1_sim_manipulation_io")
            self.declare_parameter("object_state_socket", "/tmp/unitree_g1_mujoco_objects.sock")
            self.declare_parameter("command_socket", "/tmp/unitree_g1_mujoco_manipulation.sock")
            self.declare_parameter(
                "command_response_socket", "/tmp/unitree_g1_mujoco_manipulation_client.sock"
            )
            self.declare_parameter("command_ack_timeout", 0.5)
            self.declare_parameter("frame_id", "odom")
            self.declare_parameter("output_topic", "/objects")
            self.declare_parameter("publish_rate_hz", 20.0)
            self.declare_parameter("stale_timeout", 0.5)
            self.declare_parameter("left_object_id", "red_cube")
            self.declare_parameter("right_object_id", "red_cube")
            self.declare_parameter('grasp_backend','dex3_contact')

            self.object_state_socket_path = str(self.get_parameter("object_state_socket").value)
            self.command_socket_path = str(self.get_parameter("command_socket").value)
            self.command_response_socket_path = str(
                self.get_parameter("command_response_socket").value
            )
            self.command_ack_timeout = float(self.get_parameter("command_ack_timeout").value)
            self.frame_id = str(self.get_parameter("frame_id").value)
            self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
            self.stale_timeout = float(self.get_parameter("stale_timeout").value)
            self.left_object_id = str(self.get_parameter("left_object_id").value)
            self.right_object_id = str(self.get_parameter("right_object_id").value)
            if not all(
                (
                    self.object_state_socket_path,
                    self.command_socket_path,
                    self.command_response_socket_path,
                )
            ):
                raise ValueError("socket paths must not be empty")
            if not self.frame_id:
                raise ValueError("frame_id must not be empty")
            if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
                raise ValueError("publish_rate_hz must be positive and finite")
            if not math.isfinite(self.stale_timeout) or self.stale_timeout < 0.0:
                raise ValueError("stale_timeout must be nonnegative and finite")
            if not math.isfinite(self.command_ack_timeout) or self.command_ack_timeout <= 0.0:
                raise ValueError("command_ack_timeout must be positive and finite")
            _validate_identifier(self.left_object_id)
            _validate_identifier(self.right_object_id)

            self._cache = StateCache()
            self._state_socket = None
            self._command_socket = None
            try:
                _remove_socket_path(self.object_state_socket_path)
                self._state_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self._state_socket.setblocking(False)
                self._state_socket.bind(self.object_state_socket_path)
                self._command_socket = CommandClient(
                    self.command_socket_path,
                    self.command_response_socket_path,
                    self.command_ack_timeout,
                )
            except Exception:
                self._close_sockets()
                raise

            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self._publisher = self.create_publisher(Detection3DArray, str(self.get_parameter("output_topic").value), qos)
            backend = self.get_parameter('grasp_backend').value
            if backend=='dex3_contact':
                from .dex3_grasp import Dex3GraspController
                self.dex3 = Dex3GraspController(self)
            elif backend=='legacy_magnetic':
                self.create_service(GraspObject, "~/grasp_object", self._grasp_object)
                self.create_service(Trigger, "~/grasp_left", self._grasp_left)
                self.create_service(Trigger, "~/grasp_right", self._grasp_right)
                self.create_service(Trigger, "~/release_left", self._release_left)
                self.create_service(Trigger, "~/release_right", self._release_right)
                self.create_service(Trigger, "~/pin_base", self._pin_base)
                self.create_service(Trigger, "~/unpin_base", self._unpin_base)
            else:
                raise ValueError('grasp_backend must be dex3_contact or explicit legacy_magnetic')
            self.create_timer(1.0 / self.publish_rate_hz, self._receive_and_publish)

        def _drain_state_socket(self, received_at: float) -> None:
            while True:
                try:
                    datagram = self._state_socket.recv(MAX_DATAGRAM_SIZE + 1)
                except BlockingIOError:
                    return
                except OSError as exc:
                    self.get_logger().error(f"object state socket receive failed: {exc}")
                    return
                try:
                    self._cache.update(parse_state_record(datagram), received_at)
                except ProtocolError as exc:
                    self.get_logger().warning(f"discarding malformed object state: {exc}")

        def _receive_and_publish(self) -> None:
            now = time.monotonic()
            self._drain_state_socket(now)
            stamp = self.get_clock().now().to_msg()
            message = Detection3DArray()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            for record in self._cache.fresh_records(now, self.stale_timeout):
                detection = Detection3D()
                detection.header.stamp = stamp
                if record.acquisition_time_ns is not None:
                    # Construct a new stamp: assigning the array header's stamp
                    # and mutating it would alias timestamps across detections.
                    detection.header.stamp = rclpy.time.Time(nanoseconds=record.acquisition_time_ns).to_msg()
                    detection.id = 'native_acquisition_v2:'+record.object_id
                detection.header.frame_id = self.frame_id
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = record.object_id
                hypothesis.hypothesis.score = 1.0
                hypothesis.pose.pose.position.x = record.position[0]
                hypothesis.pose.pose.position.y = record.position[1]
                hypothesis.pose.pose.position.z = record.position[2]
                hypothesis.pose.pose.orientation.x = record.orientation[0]
                hypothesis.pose.pose.orientation.y = record.orientation[1]
                hypothesis.pose.pose.orientation.z = record.orientation[2]
                hypothesis.pose.pose.orientation.w = record.orientation[3]
                detection.results.append(hypothesis)
                detection.bbox.center.position.x = record.position[0]
                detection.bbox.center.position.y = record.position[1]
                detection.bbox.center.position.z = record.position[2]
                detection.bbox.center.orientation.x = record.orientation[0]
                detection.bbox.center.orientation.y = record.orientation[1]
                detection.bbox.center.orientation.z = record.orientation[2]
                detection.bbox.center.orientation.w = record.orientation[3]
                detection.bbox.size.x = record.size[0]
                detection.bbox.size.y = record.size[1]
                detection.bbox.size.z = record.size[2]
                message.detections.append(detection)
            self._publisher.publish(message)

        def _send_command(self, command: bytes) -> tuple[bool, str]:
            return self._command_socket.send(command)

        def _send_grasp(self, side: str, object_id: str) -> tuple[bool, str]:
            """Internal object-selectable grasp API for protocols beyond gate 2."""
            try:
                command = format_manipulation_command("GRASP", side, object_id)
            except ProtocolError as exc:
                return False, str(exc)
            return self._send_command(command)

        def _send_release(self, side: str) -> tuple[bool, str]:
            try:
                command = format_manipulation_command("RELEASE", side)
            except ProtocolError as exc:
                return False, str(exc)
            return self._send_command(command)

        def _send_base_pin(self, pin: bool) -> tuple[bool, str]:
            return self._send_command(format_base_pin_command(pin))

        @staticmethod
        def _set_response(response, result: tuple[bool, str]):
            response.success, response.message = result
            return response

        def _grasp_left(self, _request, response):
            return self._set_response(response, self._send_grasp("left", self.left_object_id))

        def _grasp_object(self, request, response):
            return self._set_response(response, self._send_grasp(request.arm, request.object_id))

        def _grasp_right(self, _request, response):
            return self._set_response(response, self._send_grasp("right", self.right_object_id))

        def _release_left(self, _request, response):
            return self._set_response(response, self._send_release("left"))

        def _release_right(self, _request, response):
            return self._set_response(response, self._send_release("right"))

        def _pin_base(self, _request, response):
            return self._set_response(response, self._send_base_pin(True))

        def _unpin_base(self, _request, response):
            return self._set_response(response, self._send_base_pin(False))

        def _close_sockets(self) -> None:
            if self._state_socket is not None:
                self._state_socket.close()
                self._state_socket = None
            if self._command_socket is not None:
                self._command_socket.close()
                self._command_socket = None
            _remove_socket_path(self.object_state_socket_path)

        def destroy_node(self):
            self._close_sockets()
            return super().destroy_node()

    return SimManipulationIO, rclpy


def main(args: Iterable[str] | None = None) -> None:
    node_class, rclpy = _create_node_class()
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
