from math import inf

import numpy as np
import igraph as ig

from local.evaluation import subdivision_evaluate
from local.interpolation import cumulative_distance
from local.neighbors import NearestNeighbors


class PRM():

    def __init__(self, state_space, state_validity_checker, interpolation_fn, params):
        self.graph = ig.Graph()
        self.state_space = state_space
        self.svc = state_validity_checker
        self.interp_fn = interpolation_fn
        self.max_iters = params.get('max_iters', 1000)
        self.k = params.get('k', 3)
        self.ball_radius = params.get('ball_radius', 2)

    def plan(self, q_start, q_goal):
        iters = 0
        self._init_roadmap(q_start, q_goal)
        while not self._success() or iters < self.max_iters:
            q_rand = self._sample()
            if self._validate(q_rand):
                self._add_vextex(q_rand)
                neighbors = self._neighbors(q_rand)
                for q_near in neighbors:
                    successful, local_path = self._extend(q_near, q_rand)
                    if successful:
                        self._add_edge_to_graph(q_near, q_rand, local_path)
                self.nn.fit()
            iters += 1
        if self._success():
            return self.best_path()
        else:
            return []

    def best_path(self):
        return self.graph.get_all_shortest_paths('start', 'goal', weights='weight')

    def _init_roadmap(self, q_start, q_goal):
        self.graph.add_vertex("start")
        # Start is always at the 0 index.
        self.graph.vs[0]["value"] = list(q_start)
        self.graph.add_vertex("goal")
        # Goal is always at the 1 index.
        self.graph.vs[1]["value"] = list(q_goal)
        self.nn = NearestNeighbors(X=np.array(
            [q_start, q_goal]), model_kwargs={"leaf_size": 40})
        # seed the roadmap with a few random samples to build enough neighbors
        while len(self.nn.X) <= self.k + 1:
            q_rand = self._sample()
            if self._validate(q_rand):
                self._add_vextex(q_rand)
        self.nn.fit()

    def _success(self):
        paths = self.graph.get_all_shortest_paths('start', 'goal')
        if len(paths) > 0 and paths[0][0] != inf:
            return True
        return False

    def _validate(self, sample):
        return self.svc.validate(sample)

    def _extend(self, q_near, q_rand):
        local_path = self.interp_fn(q_near, q_rand, steps=50)
        valid = subdivision_evaluate(self.svc.validate, local_path)
        if valid:
            return True, local_path
        else:
            return False, []
        
    def _neighbors(self, sample):
        distances, neighbors = self.nn.query(sample, k=self.k)
        return [point for idx, point in enumerate(neighbors) if distances[idx] <= self.ball_radius]

    def _sample(self):
        return np.array(self.state_space.sample())

    def _add_vextex(self, sample):
        self.nn.append(sample)
        self.graph.add_vertex(None, **{'value': list(sample)})

    def _add_edge_to_graph(self, q_near, q_sample, local_path):
        self.graph.add_edge(self._idx_of_point(q_near), self._idx_of_point(
            q_sample), **{'weight': self._weight(local_path)})

    def _weight(self, local_path):
        return cumulative_distance(local_path)

    def _idx_of_point(self, point):
        return self.graph.vs['value'].index(list(point))
