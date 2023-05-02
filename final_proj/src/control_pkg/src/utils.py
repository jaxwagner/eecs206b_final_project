import numpy as np

def log_map(R):

    phi = np.arccos((np.trace(R) -1)/2)

    log_R = phi / (2*np.sin(phi) + 0.001) * (R - R.T)

    w = np.array([-log_R[1,2], log_R[0,2], -log_R[0,1]])

    return w

def hat_map(w):
    w = w.reshape((-1,))
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]

    mat = np.array([[0, -w3, w2],
                    [w3, 0, -w1],
                    [-w2, w1, 0]])

    return mat