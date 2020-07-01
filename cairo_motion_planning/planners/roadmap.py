import numpy as np
import igraph as ig

from local.evaluation import subdivision_evaluate
from local.interpolation import cumulative_distance
from local.neighbors import NearestNeighbors


class PRM():

    def __init__(self, state_space, sampler, state_validity_checker, interpolation_fn, iterations):
        self.graph = ig.Graph()
        self.sampler = sampler
        self.svc = state_validity_checker
        self.interp_fn = interpolation_fn
        self.iterations = iterations

    def plan(self, q_start, q_goal):
        iters = 0
        self._init_graph(q_start, q_goal)
        while not self._success() or iters < self.iterations:
            iters += 1
            q_rand = self._sample()
            if self._validate(q_rand):
                neighbors = self.nn.query(q_rand, k=3)
                refit_nn = False
                # If a random sample is successfully reached by from one of the nearest neighbors, it will be added to the graph. It does not need to be re-added more than once.
                added_to_graph = False 
                for q_near in neighbors:
                    successful, local_path = self._extend(q_near, q_rand)
                    if successful:
                        if not added_to_graph:
                            self._add_vextex_to_graph(q_rand)
                            added_to_graph = True
                            self.nn.append(q_rand)
                            refit_nn = True
                        self._add_edge_to_graph(q_near, q_rand, local_path)
                if refit_nn:
                    self.nn.fit()
        if self._success():
            return self.best_path()
        else:
            return []
    
    def best_path(self):
        return self.graph.shortest_paths_dijkstra([0], [1], weights="weight")
        
    def _init_graph(self, q_start, q_goal):
        self.graph.add_vertex("start")
        # Start is always at the 0 index.
        self.graph.vs[0]["q"] = q_start
        self.graph.add_vertex("goal")
        # Goal is always at the 1 index.
        self.graph.vs[1]["q"] = q_goal
        self.nn = NearestNeighbors(X=np.array(
            [q_start, q_goal]), model_kwargs={"leaf_size": 40})

    def _weight(self, local_path):
        return cumulative_distance(local_path)

    def _validate(self, sample):
        return self.svc.validate(sample)

    def _success(self):
        paths = self.graph.shortest_paths_dijkstra([0], [1], weights="weight")
        if len(paths) > 0:
            return True
        return False

    def _extend(self, q_near, q_rand):
        local_path = self.interp_fn(q_near, q_rand)
        valid = subdivision_evaluate(self.svc.validate, local_path)
        if valid:
            return True, local_path
        else:
            return False, []
        
    def _sample(self):
        return self.sampler.sample()        
    
    def _add_vextex_to_graph(self, sample):
        self.graph.add_vertex(None, **{'value': list(q_rand)})

    def _add_edge_to_graph(self, q_near, q_sample, local_path):
        pass
    
    def _idx_of_point(self, point):
        return self.graph.vs['value'].index(list(point))