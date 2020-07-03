from functools import partial

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle

from geometric.state_space import R2
from sampling.state_validity import StateValidityChecker
from local.interpolation import parametric_lerp
from planners.roadmap import PRM

import numpy as np


def minjerk_coefficients(points_array, duration_array=None):
     """
     Compute the min-jerk coefficients for a given set for user-supplied control pts
     
     params:
        points_array: array of user-supplied control points
            numpy.array of size N by k
            N is the number of control points
            k is the number of dimensions for each point
        duration_array: array of user-supplied control duration of ech segment
            numpy.array of size N-1
            N is the number of control points
     returns:
       m_coeffs:  k-dimensional array of N-1 x (6 coefficients + 1 duration of each segment)
            numpy.array of size N-1 by (6+1) by k
     """
     (rows, k) = np.shape(points_array)
     N = rows - 1  # N minus 1 because points array includes x_0
     m_coeffs = np.zeros(shape=(k, N, 7))
     x = points_array[0]
     v = np.zeros(k)
     a = np.zeros(k)
     if duration_array == None:
          duration_array = np.array([1.0]*N)
     assert len(duration_array) == N,\
          "Invalid number of intervals chosen (must be equal to N+1={})".format(N)
     for i in range(0, N):
          gx = points_array[i+1];
          t = duration_array[i]
          if i == N-1:
               gv = np.zeros(k)
          else:
               t0 = t
               t1 = duration_array[i+1]
               d0 = points_array[i+1] - points_array[i]
               d1 = points_array[i+2] - points_array[i+1]
               v0 = d0 / t0
               v1 = d1 / t1
               gv = np.where(np.multiply(v0, v1)>=1e-10, 0.5 * ( v0 + v1 ), np.zeros(k)) # 0 + eps
          ga = np.zeros(k)

          A=(gx-(x+v*t+(a/2.0)*t*t))/(t*t*t);
          B=(gv-(v+a*t))/(t*t);
          C=(ga-a)/t;

          a0=x;
          a1=v;
          a2=a/2.0;
          a3=10*A-4*B+0.5*C;
          a4=(-15*A+7*B-C)/t;
          a5=(6*A-3*B+0.5*C)/(t*t);

          x = gx
          v = gv

          m_coeffs[:,i,0] = a0
          m_coeffs[:,i,1] = a1
          m_coeffs[:,i,2] = a2
          m_coeffs[:,i,3] = a3
          m_coeffs[:,i,4] = a4
          m_coeffs[:,i,5] = a5
          m_coeffs[:,i,6] = t
     return m_coeffs

def minjerk_trajectory(m_coeffs, num_intervals, duration_array=None):
    """
    Iterpolation of the entire minimum jerk trajectory at once,
    using a specified number of intervals between
    control points (encapsulated by m_coeffs).
    params:
        m_coeffs: N-dimensional array of (6+1) x k  coefficients
            for every control point
            numpy.array of size N by (6 + 1) by k
            N is the number of control points
            k is the number of dimensions for each point
        num_intervals: the number of intervals between
            control points
            int > 0
        duration_array: array of user-supplied control duration of segment
            numpy.array of size N-1
            N is the number of control points
    returns:
        m_curve: positions along the minimum trajectory  in k-dimensions
            numpy.array of size N*num_interval+1  by k
            (the +1 is to include the start position on the curve)
    """
    assert num_intervals > 0,\
        "Invalid number of intervals chosen (must be greater than 0)"
    interval = 1.0 / num_intervals
    (num_axes, num_mpts, _) = np.shape(m_coeffs)
    m_curve = np.zeros((num_mpts*num_intervals+1, num_axes))
    # Copy out initial point
    m_curve[0, :] = m_coeffs[:, 0, 0]
    if duration_array == None:
         duration_array = np.array([1.0]*num_mpts)
    assert len(duration_array) == num_mpts,\
         "Invalid number of intervals chosen (must be equal to N={})".format(num_mpts)
    for current_mpt in range(num_mpts):
         m_coeff_set = m_coeffs[:, current_mpt, range(7)]
         for iteration, t in enumerate(np.linspace(interval, 1,
                                                   num_intervals)):
              m_curve[(current_mpt *
                       num_intervals +
                       iteration+1), :] = _minjerk_trajectory_point(m_coeff_set, t * duration_array[current_mpt])
    return m_curve
    
def _minjerk_trajectory_point(m_coeff, t):
    """
    Internal convenience function for calculating
    a k-dimensional point defined by the supplied
    minimum jerk coefficients. Finds the point that
    describes the current position along the minimum
    trajectory segment for k dimensions.
    params:
        m_coeff => m0...m3: Four k-dimensional minimum jerk
            coefficients each one is a numpy.array
            of size k by 1, so
            m_coeff is a numpy array of size k by (6+1)
            k is the number of dimensions for each
            coefficient
        t: percentage of time elapsed for this segment
            0 <= int <= 1.0
    returns:
        current position in k dimensions
            numpy.array of size 1 by k
    """
    a0 = m_coeff[:,0]
    a1 = m_coeff[:,1]
    a2 = m_coeff[:,2]
    a3 = m_coeff[:,3]
    a4 = m_coeff[:,4]
    a5 = m_coeff[:,5]
    tm = m_coeff[:,6]

    t = t * tm # input t is percentage of time elapsed for this segment, tm is the duration of this segment and to calculate x, v, a , t is the time[s] elapsed for this segment

    # calculate x, v, z at the time percentage  t
    # x=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t;
    x=a0+a1*t+a2*np.power(t,2)+a3*np.power(t,3)+a4*np.power(t,4)+a5*np.power(t,5);
    # v=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t;
    v=a1+2*a2*t+3*a3*np.power(t,2)+4*a4*np.power(t,3)+5*a5*np.power(t,4);
    # a=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t;
    a=2*a2+6*a3*t+12*a4*np.power(t,2)+20*a5*np.power(t,3);

    return x

def minjerk_point(m_coeffs, m_index, t):
    """
    Finds the k values that describe the current
    position along the minjerk trajectory for k dimensions.
    params:
        m_coeffs: k-dimensional array
            for every control point with 6 Minimum Jerk coefficients and a segument duration
            numpy.array of size k by N by 7
            N is the number of control points
            k is the number of dimensions for each point
        m_index: index position out between two of
            the N b_coeffs for this point in time
            int
        t: percentage of time that has passed between
            the two control points
            0 <= int <= 1.0
    returns:
        m_point: current position in k dimensions
            numpy.array of size 1 by k
    """
    if m_index <= 0:
        m_point = m_coeffs[:, 0, 0]
    elif m_index > m_coeffs.shape[1]:
        t = 1
        m_coeff_set = m_coeffs[:,m_coeffs.shape[1]-1, range(7)]
        m_point = _minjerk_trajectory_point(m_coeff_set, t)
    else:
        t = 0.0 if t < 0.0 else t
        t = 1.0 if t > 1.0 else t
        m_coeff_set = m_coeffs[:,m_index-1, range(7)]
        m_point = _minjerk_trajectory_point(m_coeff_set, t)
    return m_point

def plot_prm(found_path, vertices, edges):
        fig, ax = plt.subplots()
        x, y = zip(*found_path)
        ax.plot(x, y, zorder=2, color='red',
                        linewidth=4, linestyle='--', label='Regular Path')
        ax.scatter(found_path[0][0], found_path[0][1],
                        color='green', s=150, zorder=3)
        ax.scatter(found_path[-1][0], found_path[-1][1], color='blue', s=150, zorder=3)
        
        line_segments = LineCollection(
            edges, colors='gray', linestyle='solid', alpha=.5, zorder=1)
        ax.add_collection(line_segments)
        
        x, y = zip(*vertices)
        ax.scatter(x, y, zorder=2, color='blue')
        
        
        ax.set_xlim([0, 10])
        ax.set_ylim([0, 10])
        b1 = Rectangle((2,2), 2, 2)
        b2 = Rectangle((6,1), 2, 9)
        patch_collection = PatchCollection([b1, b2], alpha=0.4)
        ax.add_collection(patch_collection)
        ax.set_title('PRM')
        ax.legend()
        plt.show()

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
    
    # This inherently uses UniformSampler but a different sampling class could be injected.
    r2_space = R2()
    
    ##############################
    # State Validity Formulation #
    ##############################
    # There is no self-collision for point object in R2
    self_col_fn = None
    
    # Create a collision function that combines two box_collision functions
    b1 = partial(box_collision, coordinates=[[2,2], [4, 4]])
    b2 = partial(box_collision, coordinates=[[6,1], [8,10]])
    col_fn = lambda sample: all([b1(sample), b2(sample)])

    # In this case, we only have a col_fn.
    svc = StateValidityChecker(self_col_func=None, col_func=col_fn, validity_funcs=None)
        
    ############################################
    # Build the PRM and call the plan function #
    ############################################
    # Create the PRM
    interp = partial(parametric_lerp, steps=50)
    prm = PRM(r2_space, svc, interp, params={'max_iters': 3000, 'k': 3})
    
    plan = prm.plan(np.array([1, 1]), np.array([9, 9]))
    
    #########################################################
    # Interpolate between each point again then use points  #
    #########################################################
    path = prm.get_path(plan)
    
    vertices = [prm.graph.vs[idx]['value'] for idx in range(0, len(prm.graph.vs))]
    edges = [(prm.graph.vs[e[0]]['value'], prm.graph.vs[e[1]]['value']) for e in prm.graph.get_edgelist()]
    control_points = np.array([np.array(point) for point in path])
    coeff = minjerk_coefficients(control_points)
    traj = minjerk_trajectory(coeff, 100)
    plot_prm(traj, vertices, edges)
    
    
