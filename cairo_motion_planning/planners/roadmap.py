import igraph as ig

from cairo_motion_planning.local.eval import SubdividionEvaluation
from neighbors import NearestNeighbors

class PRM():

    def __init__(self, state_space, state_validity_checker):
        self.graph = ig.Graph()
        # self.edge_props = EdgePropertyMap()
        # self.vertex_props = VertexPropertyMap()
        # self.sampler = sampler
        self.state_validity_checker = state_validity_checker

    def plan(self, q_start, q_goal):

        while not path(q_start, q_goal):
            q_rand = sample()
            neighbors = get_neighbors(G, q_rand) # might require updating neighbors during each iteration?
            for q_near in neighbors:
                if connect(q_near, q_rand):
                    graph.add(q_near,q_rand)

    def _init_graph(self, q_start, q_goal):
        self.graph.add_vertex("start")
        self.graph.vs[0]["q"] = q_start
        self.graph.add_vertex("goal")
        self.graph.vs[1]["q"] = q_goal
