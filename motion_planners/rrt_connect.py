import time

from itertools import takewhile

from .meta import direct_path, direct_path_force_aware
from .smoothing import smooth_path, smooth_path_old, smooth_path_force_aware
from .rrt import TreeNode, configs, derivatives
from .utils import irange, argmin, RRT_ITERATIONS, RRT_RESTARTS, RRT_SMOOTHING, INF, elapsed_time, \
    negate
import numpy as np


def asymmetric_extend(q1, q2, extend_fn, backward=False):
    if backward:
        return reversed(list(extend_fn(q2, q1)))
    return extend_fn(q1, q2)

def extend_towards(tree, target, distance_fn, extend_fn, collision_fn, swap, tree_frequency):
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    extend = list(asymmetric_extend(last.config, target, extend_fn, swap))
    safe = list(takewhile(negate(collision_fn), extend))
    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success


def extend_towards_force_aware(tree, target, distance_fn, extend_fn, collision_fn, torque_fn, dynam_fn, swap, tree_frequency):
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    extend = list(asymmetric_extend(last.config, target, extend_fn, swap))
    safe = list(takewhile(negate(collision_fn), extend))
    safe = list(takewhile(torque_fn, safe))
    print("number of safe extended path:", len(safe))
    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success

def rrt_connect(q1, q2, distance_fn, sample_fn, extend_fn, collision_fn,
                iterations=RRT_ITERATIONS, tree_frequency=1, max_time=INF):
    start_time = time.time()
    assert tree_frequency >= 1
    if collision_fn(q1) or collision_fn(q2):
        return None
    nodes1, nodes2 = [TreeNode(q1)], [TreeNode(q2)]
    for iteration in irange(iterations):
        if max_time <= elapsed_time(start_time):
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        last1, _ = extend_towards(tree1, sample_fn(), distance_fn, extend_fn, collision_fn,
                                  swap, tree_frequency)
        last2, success = extend_towards(tree2, last1.config, distance_fn, extend_fn, collision_fn,
                                        not swap, tree_frequency)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            #print('{} iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            return configs(path1[:-1] + path2[::-1])
    return None

#################################################################

def birrt(q1, q2, distance, sample, extend, collision,
          restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING, max_time=INF, **kwargs):
    # TODO: move to the meta class
    start_time = time.time()
    if collision(q1) or collision(q2):
        return None
    path = direct_path(q1, q2, extend, collision)
    if path is not None:
        return path
    for attempt in irange(restarts + 1):
        # TODO: use the restart wrapper
        if max_time <= elapsed_time(start_time):
            break
        path = rrt_connect(q1, q2, distance, sample, extend, collision,
                           max_time=max_time - elapsed_time(start_time), **kwargs)
        if path is not None:
            #print('{} attempts'.format(attempt))
            if smooth is None:
                return path
            #return smooth_path_old(path, extend, collision, iterations=smooth)
            return smooth_path(path, extend, collision, distance_fn=distance, iterations=smooth,
                               max_time=max_time - elapsed_time(start_time))
    return None


def rrt_connect_force_aware(q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, torque_fn, dynam_fn,
                iterations=RRT_ITERATIONS, tree_frequency=1, smooth = None, max_time=INF):
    start_time = time.time()
    assert tree_frequency >= 1
    if collision_fn(q1) or collision_fn(q2):
        return None, None, None
    if not torque_fn(q1) or not torque_fn(q2):
        return None, None, None
    nodes1, nodes2 = [TreeNode(q1)], [TreeNode(q2)]
    for iteration in irange(iterations):
        if max_time <= elapsed_time(start_time):
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        last1, _ = extend_towards_force_aware(tree1, sample_fn(), distance_fn, extend_fn, collision_fn, torque_fn, dynam_fn,
                                  swap, tree_frequency)
        last2, success = extend_towards_force_aware(tree2, last1.config, distance_fn, extend_fn, collision_fn, torque_fn, dynam_fn,
                                        not swap, tree_frequency)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            #print('{} iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            rrtPath = np.array(configs(path1[:-1] + path2[::-1]))
            vels = []
            accels = []
            path = rrtPath
            if not smooth:
                path, _, vels, accels = dynam_fn(rrtPath, len(rrtPath))
                if path is None:
                    return None, None, None
                for i in range(len(path)):
                    if not torque_fn(path[i], velocities=vels[i], accelerations=accels[i]):
                        return None, None, None
            return path, vels, accels
    return None, None, None

#################################################################

def birrt_force_aware(q1, q2, distance, sample, extend, collision, torque, dynam,
          restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING, max_time=INF, **kwargs):
    # TODO: move to the meta class
    start_time = time.time()
    if collision(q1) or collision(q2):
        return None, None, None
    if not torque(q1) or not torque(q2):
        return None, None, None
    path = direct_path_force_aware(q1, q2, extend, collision, torque, dynam)
    # path = None
    if path is not None:
        return path
    for attempt in irange(restarts + 1):
        # TODO: use the restart wrapper
        if max_time <= elapsed_time(start_time):
            break
        path, vels, accels  = rrt_connect_force_aware(q1, q2, distance, sample, extend, collision, torque, dynam,
                           max_time=max_time - elapsed_time(start_time), smooth=smooth, **kwargs)
        if path is not None:
            #print('{} attempts'.format(attempt))
            if smooth is None:
                return path, vels, accels
            #return smooth_path_old(path, extend, collision, iterations=smooth)
            return smooth_path_force_aware(path, extend, collision, torque_fn=torque, dynam_fn=dynam, distance_fn=distance, iterations=smooth,
                               max_time=max_time - elapsed_time(start_time))
    return path, vels, accels

def rrt_star_force_aware(q1, q2, distance, sample, extend, collision, torque, dynam,
          restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING, max_time=INF, **kwargs):
    start_time = time.time()
    if collision(q1) or collision(q2):
        return None, None, None
    if not torque(q1) or not torque(q2):
        return None, None, None