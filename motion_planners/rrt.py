from random import random

from .utils import irange, argmin, RRT_ITERATIONS


class TreeNode(object):

    def __init__(self, config, parent=None, velocities=None, accelerations=None):
        self.config = config
        self.parent = parent
        self.velocities = velocities
        self.accelerations = accelerations
        if velocities == None or accelerations==None:
            self.velocities = [0.0]*len(config)
            self.accelerations=[0.0]*len(config)

    #def retrace(self):
    #    if self.parent is None:
    #        return [self]
    #    return self.parent.retrace() + [self]

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def draw(self, env, color=(1, 0, 0, .5)):
        from manipulation.primitives.display import draw_node, draw_edge
        self.node_handle = draw_node(env, self.config, color=color)
        if self.parent is not None:
            self.edge_handle = draw_edge(
                env, self.config, self.parent.config, color=color)

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
    __repr__ = __str__


def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))

def derivatives(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.velocities, nodes)), list(map(lambda n: n.accelerations, nodes))


def rrt(start, goal_sample, distance, sample, extend, collision, goal_test=lambda q: False,
        iterations=RRT_ITERATIONS, goal_probability=.2):
    if collision(start):
        return None
    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g
    nodes = [TreeNode(start)]
    for i in irange(iterations):
        goal = random() < goal_probability or i == 0
        s = goal_sample() if goal else sample()

        last = argmin(lambda n: distance(n.config, s), nodes)
        for q in extend(last.config, s):
            if collision(q):
                break
            last = TreeNode(q, parent=last)
            nodes.append(last)
            if goal_test(last.config):
                return configs(last.retrace())
        else:
            if goal:
                return configs(last.retrace())
    return None
