'''
Some utilities for LEAP Hand that help with converting joint angles between each convention.
'''
import numpy as np


'''
Embodiments:

LEAPhand: Real LEAP hand (180 for the motor is actual zero)

'''


###Sometimes it's useful to constrain the thumb more heavily(you have to implement here), but regular usually works good.
def LEAP_limits(hand="right"):
    # Finger joints (0-11) are identical for left and right
    # Thumb joints (12-15) differ: joints 12-13 are negated/swapped for left hand
    if hand == "left":
        # Left hand: thumb limits from leap_left.urdf
        sim_min = np.array([-0.523599, -0.261799, -0.261799, -0.261799,-0.523599, -0.261799, -0.261799, -0.261799, -0.523599, -0.261799, -0.261799, -0.261799, -2.094, -2.443, -1.20, -1.34])
        sim_max = np.array([0.523599, 1.91986, 1.91986, 1.91986,  0.523599, 1.91986, 1.91986, 1.91986,  0.523599, 1.91986, 1.91986, 1.91986,  0.349,  0.47, 1.90,  1.88])
    else:
        sim_min = np.array([-0.523599, -0.261799, -0.261799, -0.261799,-0.523599, -0.261799, -0.261799, -0.261799, -0.523599, -0.261799, -0.261799, -0.261799, 0.0, 0.0, -0.349066, -0.349066])
        sim_max = np.array([0.523599, 1.91986, 1.91986, 1.91986,  0.523599, 1.91986, 1.91986, 1.91986,  0.523599, 1.91986, 1.91986, 1.91986,  1.571,  1.571, 1.74533,  1.74533])
    return sim_min, sim_max

#this goes from [-1, 1] to [lower, upper]
def scale(x, lower, upper):
    return (0.5 * (x + 1.0) * (upper - lower) + lower)
#this goes from [lower, upper] to [-1, 1]
def unscale(x, lower, upper):
    return (2.0 * x - upper - lower)/(upper - lower)

