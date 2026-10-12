import math
from types import SimpleNamespace as S

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.exploration_search import ObservedSearchMap


def grid(data,resolution=.2,yaw=0.):
    return S(data=np.asarray(data).ravel().tolist(),info=S(width=len(data[0]),height=len(data),resolution=resolution,
        origin=S(position=S(x=-2.,y=-2.),orientation=S(w=math.cos(yaw/2),x=0.,y=0.,z=math.sin(yaw/2)))))


def test_goals_have_known_free_footprints_and_reachable_paths():
    data = np.zeros((40,50),dtype=int)
    data[:,25] = 100  # Disconnected visible room must not yield a goal.
    data[:,40:] = -1
    observed = ObservedSearchMap(grid(data),.4)
    robot = observed.world(10,15)
    goals = observed.viewpoints(robot)
    assert goals
    reachable = observed.reachable(robot,35.)
    for goal in goals:
        x,y = observed.cell((goal.x,goal.y))
        assert x<25 and observed.free[y,x] and reachable[y,x]>=0
        assert np.all(data[y-2:y+3,x-2:x+3]==0)


def test_fully_mapped_space_still_has_visual_search_viewpoints():
    observed = ObservedSearchMap(grid(np.zeros((40,40),dtype=int)),.4)
    robot = observed.world(15,15)
    goals = observed.viewpoints(robot,[robot])
    assert goals and all(goal.kind=='visual_coverage' for goal in goals)
    blocked = [(goal.x,goal.y) for goal in goals]
    assert not observed.viewpoints(robot,[robot],blocked)


def test_unknown_space_is_not_a_traversable_shortcut_and_rays_stop_at_walls():
    data = np.zeros((40,40),dtype=int)
    data[:,19:22] = -1
    observed = ObservedSearchMap(grid(data),.2)
    goals = observed.viewpoints(observed.world(8,15))
    assert goals and all(observed.cell((goal.x,goal.y))[0]<19 for goal in goals)
    data[:,18] = 100
    observed = ObservedSearchMap(grid(data),.2)
    assert observed.information(8,15)==0


def test_rotated_map_coordinates_and_people_admission():
    observed = ObservedSearchMap(grid(np.zeros((40,40),dtype=int),yaw=.7),.4)
    assert observed.cell(observed.world(13,17)) == (13,17)
    robot = observed.world(15,15)
    first = observed.viewpoints(robot)[0]
    goals = observed.viewpoints(robot,people=[(first.x,first.y,0.,0.,.4)])
    assert all(math.dist((goal.x,goal.y),(first.x,first.y))>=1.2 for goal in goals)


def test_invalid_and_too_small_maps_fail_closed():
    with pytest.raises(ValueError):
        ObservedSearchMap(grid([[0]],resolution=0.))
    observed = ObservedSearchMap(grid([[0]],resolution=.01),radius=.55)
    assert not observed.viewpoints((0.,0.))


def test_batched_information_matches_first_hit_ray_reference():
    rng = np.random.default_rng(4)
    data = rng.choice([0,-1,100],size=(30,40),p=[.8,.1,.1])
    observed = ObservedSearchMap(grid(data),0.)
    cells = [(0,0),(12,14),(39,29),(20,20)]
    expected = []
    for x,y in cells:
        boundary = set()
        for angle in np.linspace(-math.pi,math.pi,32,endpoint=False):
            for step in range(1,math.ceil(3./observed.resolution)+1):
                nx,ny = round(x+math.cos(angle)*step),round(y+math.sin(angle)*step)
                if not (0<=nx<observed.width and 0<=ny<observed.height):
                    break
                if data[ny,nx]<0:
                    boundary.add((nx,ny))
                    break
                if data[ny,nx]>=20:
                    break
        expected.append(len(boundary))
    assert observed.information_batch(cells)==expected


def test_semantic_memory_only_proposes_reachable_standoff_views():
    data = np.zeros((40,40),dtype=int)
    observed = ObservedSearchMap(grid(data),.4)
    robot,target = observed.world(10,10),observed.world(25,25)
    goals = observed.semantic_viewpoints(robot,target)
    assert goals
    for goal in goals:
        assert goal.kind=='semantic_reobserve'
        assert math.dist((goal.x,goal.y),target)==pytest.approx(1.4)
        assert observed.free[observed.cell((goal.x,goal.y))[1],observed.cell((goal.x,goal.y))[0]]


def test_equivalent_views_prefer_current_heading_instead_of_unnecessary_u_turns():
    observed = ObservedSearchMap(grid(np.zeros((40,40),dtype=int)),.4)
    robot = observed.world(15,15)
    assert observed.viewpoints(robot,heading=0.)[0].x>robot[0]
    assert observed.viewpoints(robot,heading=math.pi)[0].x<robot[0]


def test_pickup_route_goes_around_partition_without_cutting_inflated_corners():
    data = np.zeros((60,60),dtype=int)
    data[15:45,30] = 100
    observed = ObservedSearchMap(grid(data,resolution=.1,yaw=.4),.3)
    start,goal = observed.world(15,30),observed.world(45,30)
    assert not observed.segment_free(start,goal)
    route = observed.route(start,goal)
    assert len(route)>=3 and route[-1]==goal
    for a,b in zip([start]+route,route):
        assert observed.segment_free(a,b)
    data[:,30] = -1
    blocked = ObservedSearchMap(grid(data,resolution=.1),.3)
    assert blocked.route(blocked.world(15,30),blocked.world(45,30))==[]


def test_route_rejects_diagonal_gap_and_large_map_has_explicit_bound():
    data = np.zeros((10,10),dtype=int)
    data[4,5] = data[5,4] = 100
    observed = ObservedSearchMap(grid(data),0.)
    assert not observed.segment_free(observed.world(4,4),observed.world(5,5))
    assert ObservedSearchMap(grid(np.zeros((480,640),dtype=int),resolution=.05)).width==640
    message = grid([[0]])
    message.info.width = 500001
    with pytest.raises(ValueError):
        ObservedSearchMap(message)


def test_unmapped_staging_uses_partial_known_free_route_not_false_arrival():
    data = np.zeros((80,100),dtype=int)
    data[:,65:] = -1
    observed = ObservedSearchMap(grid(data,resolution=.1),.55)
    start,goal = observed.world(20,40),observed.world(80,40)
    route,complete = observed.route_towards(start,goal)
    assert route and not complete and route[-1]!=goal
    assert math.dist(route[-1],goal)<math.dist(start,goal)
    for a,b in zip([start]+route,route):
        assert observed.segment_free(a,b)


def test_large_atrium_candidate_budget_includes_distant_views():
    observed = ObservedSearchMap(grid(np.zeros((120,160),dtype=int),resolution=.1),.55)
    robot = observed.world(60,60)
    goals = observed.viewpoints(robot,limit=40)
    assert len(goals)<=40
    assert max(goal.path_length for goal in goals)>8.
