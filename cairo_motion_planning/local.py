import numpy as np


def interpolate_5poly(q0, q1, tv, qd0=None, qd1=None):
    """
    This function produces a joint space trajectory qt (MxN) where the joint
    coordinates vary from q0 (1xN) to q1 (1xN).  A quintic (5th order) polynomial is used 
    with default zero boundary conditions for velocity and acceleration.  
    Time is assumed to vary from 0 to 1 in M (tv) steps.  Joint velocity and 
    acceleration can be optionally returned as qdt (MxN) and qddt (MxN) respectively.
    The trajectory qt, qdt and qddt are MxN matrices, with one row per time step,
    and one column per joint.
    
    Interpolating between points can be 
    
    The code in this function was adopted from the Robotics Toolbox jtraj function.
    
    Copyright (C) 1993-2017, by Peter I. Corke
    
    Args:
        q0 (ndarray): 1xN starting configuration vector
        q1 (ndarray): 1xN ending configuration vector
        tv (ndarray or int): Timesteps
        qd0 (ndarray, optional): Initial velocity
        qd1 (ndarray, optional): Final velocity
    
    Returns:
        ndarray, ndarray, ndarray: MXN matrix of positions, velocities, and acclerations at each time step.
    """
    # Normalize time steps either given the number of steps (int) 
    if type(tv) is list and len(tv) > 1:
        # Get the max timescale
        timescale = max(tv);
        # divide all times by the max to normalize.
        t = tv / timescale;
    else:
        timescale = 1
        t = [x / (tv - 1) for x in range(0, tv)]  # % normalized time from 0 -> 1

    q0 = q0
    q1 = q1

    if qd0 is None and qd1 is None:
        qd0 = np.zeros(np.size(q0));
        qd1 = qd0;
    else:
        qd0 = qd0
        qd1 = qd1
    # compute the polynomial coefficients
    A = 6 * (q1 - q0) - 3 * (qd1 + qd0) * timescale
    B = -15 * (q1 - q0) + (8 * qd0 + 7 * qd1) * timescale
    C = 10 * (q1 - q0) - (6 * qd0 + 4 * qd1) * timescale
    E = qd0 * timescale  # as the t vector has been normalized
    F = q0

    tt = np.array([np.power(t, 5), np.power(t, 4), np.power(t, 3), np.power(t, 2), t, np.ones(np.size(t))])
    c = np.array([A, B, C, np.zeros(np.size(A)), E, F])
    qt = tt.T.dot(c)


    # compute velocity
    c = np.array([np.zeros(np.size(A)), 5 * A, 4 * B, 3 * C,  np.zeros(np.size(A)), E])
    qdt = tt.T.dot(c) / timescale;

    # compute acceleration
    c = np.array([np.zeros(np.size(A)), np.zeros(np.size(A)), 20 * A, 12 * B, 6 * C,  np.zeros(np.size(A))])
    qddt = tt.T.dot(c) / np.power(timescale, 2);

    return qt, qdt, qddt

class SubdivisionPathIterator():

    def __init__(self, local_path):
        self.segment_queue=[local_path]

    def __iter__(self):
        return self

    def __next__(self):
        if len(self.segment_queue) == 0:
            raise StopIteration
        else:
            segment=self.segment_queue.pop(0)
            m_idx=int(len(segment) / 2)
            s1=segment[:m_idx]
            s2=segment[m_idx + 1:]
            if len(s1) > 0:
                self.segment_queue.append(s1)
            if len(s2) > 0:
                self.segment_queue.append(s2)
            if len(segment) > 1:
                return segment[m_idx]
            elif len(segment) == 1:
                return segment[0]

    next=__next__  # python2.x compatibility.


class IncrementalEvaluation():

    def __init__(self, eval_fn):
        self.eval_fn=eval_fn

    def evaluate(self, local_path):
        for point in local_path:
            if not self.eval_fun(point):
                return False
        return True


class SubdividionEvaluation():

    def __init__(self, eval_fn):
        self.eval_fn=eval_fn

    def evaluate(self, local_path):
        for point in SubdivisionPathIterator(local_path):
            if not self.eval_fun(point):
                return False
        return True
