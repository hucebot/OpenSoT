from pyopensot.lie_utils import *
import numpy as np


ksi = np.random.rand(6)

print(J_l6_inv(ksi))

print(J_r6_inv(ksi))

print(Exp6(ksi))

print(J_l6_inv(ksi)* Adjoint(Exp6(ksi)))