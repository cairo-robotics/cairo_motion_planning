from functools import partial

import numpy as np

from geometric.state_space import R2
from sampling.state_validity import StateValidityChecker
from local.interpolation import parametric_lerp
from planners.roadmap import PRM


def box_collision(sample, coordinates=[[0,0], [2, 2]]):
    x_valid = False
    y_valid = False
    if sample[0] > coordinates[0][0] and sample[0] < coordinates[1][0]:
        x_valid = True
    if sample[1] > coordinates[0][1] and sample[1] < coordinates[1][1]:
        y_valid = True
    if x_valid and y_valid:
        return False
    else:
        return True
    

if __name__ == "__main__":
    
    #########################
    # State space selection #
    #########################
    r2_space = R2()
    
    
    ##############################
    # State Validity Formulation #
    ##############################
    # There is no self-collision for point object in R2
    self_col_fn = None
    
    # Create a collision function that combines two box_collision functions
    b1 = partial(box_collision, coordinates=[[2,2], [4, 4]])
    b2 = partial(box_collision, coordinates=[[6,6], [8,8]])
    col_fn = lambda sample: all([b1(sample), b2(sample)])
    # In this case, we only have a col_fn.
    svc = StateValidityChecker(self_col_func=None, col_func=col_fn, validity_funcs=None)
    
    
    # Create the PRM
    prm = PRM(r2_space, svc, parametric_lerp, params={'max_iters': 1000, 'k': 3})
    
    print(prm.plan(np.array([1, 1]), np.array([9, 9])))
    