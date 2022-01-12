import numpy as np

l = 0.5
dt = 0.1
def BW_kinematics(confgs):
    vn = [0]
    deltan = [0]
    for Xn in range(len(confgs)-1):
        x_dot = confgs[Xn+1][0] - confgs[Xn][0]
        theta_dot = confgs[Xn+1][2] - confgs[Xn][2]

        if x_dot == 0:
            vn.append(0)
        else:
            vn.append(np.cos(confgs[Xn][2])/(x_dot))


        if vn[Xn] == 0:
            deltan.append(0)
        else:
            delta = np.arctan(l*theta_dot/vn[Xn])
            deltan.append(delta)

    inputs = list(zip(vn, deltan))
    return inputs

def FW_kinematics(inputs):
    xn = [50]
    yn = [50]
    for i in range(len(inputs) - 1):
        theta = (inputs[i][0] / l) * np.tan(inputs[i][1])

        xn.append(inputs[i][0] * np.cos(theta))
        yn.append(inputs[i][0] * np.sin(theta))

    states = list(zip(xn, yn))
    return states