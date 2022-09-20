import time

from .smoothing import smooth_path
from .utils import RRT_RESTARTS, RRT_SMOOTHING, INF, irange, elapsed_time, compute_path_cost, traverse


def direct_path(q1, q2, extend_fn, collision_fn):
    # TODO: version which checks whether the segment is valid
    if collision_fn(q1) or collision_fn(q2):
        return None
    path = [q1] + list(extend_fn(q1, q2))
    if any(collision_fn(q) for q in traverse(path)):
        return None
    return path
    # path = [q1]
    # for q in extend_fn(q1, q2):
    #     if collision_fn(q):
    #         return None
    #     path.append(q)
    # return path

def direct_path_force_aware(q1, q2, extend_fn, collision_fn, torque_fn, dynam_fn):
    # TODO: version which checks whether the segment is valid
    if collision_fn(q1) or collision_fn(q2):
        return None, None, None
    if not torque_fn(q1) or not torque_fn(q2):
        print('torque fn failed in direct path1')
        return None, None, None
    path = [q1] + list(extend_fn(q1, q2))
    vels1 = [[0.0]*len(q1)]
    accels1 = [[0.0]*len(q1)]
    vels2 = [[0.0]*len(q1)]
    accels2 = [[0.0]*len(q1)]
    for i in range(1,len(path)//2):
        vel1, acc1 = dynam_fn(path[i], path[i-1], vels1[-1], accels1[-1])
        vels1.append(vel1)
        accels1.append(acc1)
    for i in range(len(path)-2, len(path)//2 -1, -1):
        vel2, acc2 = dynam_fn(path[i], path[i+1], vels2[0], accels2[0])
        vels2 = [vel2] + vels2
        accels2 = [acc2] + accels2
    vels = vels1 + vels2
    accels = accels1 + accels2
    if any(collision_fn(q) for q in traverse(path)):
        return None, None, None
    for i in range(len(path)):
        if not torque_fn(path[i], velocities=vels[i], accelerations=accels[i]):
            print('torque fn failed in direct path2')
            return None, None, None
    return path, vels, accels

def random_restarts(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn,
                    restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                    success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    start_time = time.time()
    solutions = []
    if any(collision_fn(q) for q in [q1, q2]):
        return solutions
    path = direct_path(q1, q2, extend_fn, collision_fn)
    if path is not None:
        solutions.append(path)

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) > max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(q1, q2, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue
        if smooth is not None:
            path = smooth_path(path, extend_fn, collision_fn, iterations=smooth)
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))

    return solutions