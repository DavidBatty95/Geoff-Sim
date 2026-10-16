"""Fixed command boundaries expressed in integer physics steps."""
from decimal import Decimal,ROUND_CEILING


def command_steps(command_duration,total_duration,timestep=.002):
    dt = Decimal(str(timestep))
    start = Decimal('5')
    end = start+Decimal(str(command_duration)) if command_duration is not None else Decimal(str(total_duration))
    return tuple(int((value/dt).to_integral_value(rounding=ROUND_CEILING)) for value in (start,end))
