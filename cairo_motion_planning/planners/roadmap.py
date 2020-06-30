import numpy as np
import igraph as ig

from local.evaluation import subdivision_evaluate
from local.neighbors import NearestNeighbors


class PRM():

    def __init__(self, state_space, state_validity_check, interpolation_fn, iterations):
        self.graph = ig.Graph()
        self.state_space = state_space
        self.svc = state_validity_check
        self.interp_fn = interpolation_fn
        self.iterations = iterations

    def plan(self, q_start, q_goal):
        iters = 0
        self._init_graph( q_start, q_goal)
        while not self._success() or iters < self.iterations:
            iters += 1
            q_rand = self.state_space.sample()
            neighbors = self.nn.query(q_rand)
            refit_nn = False
            for q_near in neighbors:
                successful, local_path = self._extend(q_near, q_rand)
                if successful:
                    self.graph.add(q_near, q_rand)
                    self.nn.append(q_rand)
                    refit_nn = True
            if refit_nn:
                self.nn.fit()
            

    def _init_graph(self, q_start, q_goal):
        self.graph.add_vertex("start")
        self.graph.vs[0]["q"] = q_start
        self.graph.add_vertex("goal")
        self.graph.vs[1]["q"] = q_goal
        self.nn = NearestNeighbors(X=np.array(
            [q_start, q_goal]), k=3, model_kwargs={"leaf_size": 40})

    def _success(self):
        path = self.graph.get_path('start', 'goal')
        if path:
            return True
        return False

    def _extend(self, q_near, q_rand):
        local_path = self.interp_fn(q_near, q_rand)
        valid = subdivision_evaluate(self.svc, local_path)
        if valid:
            return True, local_path
        else:
            return False, []
