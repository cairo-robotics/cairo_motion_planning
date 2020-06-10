from local import interpolate_5poly
import numpy as np

if __name__ == "__main__":

    q0 = np.array([1, 1, 1])
    q1 = np.array([2, 2, 2])

    qt, qdt, qddt = interpolate_5poly(q0, q1, 5)

    print(qt)

    print(qdt)

    print(qddt)