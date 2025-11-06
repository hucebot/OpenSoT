from pyopensot import AffineHelper, OptvarHelper
import numpy as np
import unittest

M1 = np.array([[1, 2, 3],[4, 5, 6]])
q1 = np.array([1, 2])
V1 = AffineHelper(M1, q1)
print(f"V1: {V1}")

M2 = 2 * np.array([[1, 2, 3],[4, 5, 6]])
q2 = 2 * np.array([1, 2])
V2 = AffineHelper(M2, q2)

V3 = V1 - V2
print(f"M3: {V3.getM()}")
print(f"q3: {V3.getq()}")

utest = unittest.TestCase()
utest.assertTrue((V3.getM() == (M1 - M2)).all())
utest.assertTrue((V3.getq() == (q1 - q2)).all())

V3.update()
utest.assertTrue((V3.getM() == (M1 - M2)).all())
utest.assertTrue((V3.getq() == (q1 - q2)).all())


v = np.array([10, 22])
V4 = V1 - v
print(f"M4: {V4.getM()}")
print(f"q4: {V4.getq()}")
utest.assertTrue((V4.getM() == M1).all())
utest.assertTrue((V4.getq() == (q1 - v)).all())

V5 = V1 + V2
utest.assertTrue((V5.getM() == (M1 + M2)).all())
utest.assertTrue((V5.getq() == (q1 + q2)).all())

variables_vec = dict()
variables_vec["qddot"] = 23;
variables_vec["f1"] = 3;
variables_vec["f2"] = 3;

variables = OptvarHelper(variables_vec)
qddot = variables.getVariable("qddot")
print(f"qddot: {qddot}")
f1 = variables.getVariable("f1")
print(f"qddot: {f1}")
print(f"size: {variables.getSize()}")
vv = variables.getAllVariables()
for v in vv:
    print(v)


vars = list()
vars.append(("x", 4))
vars.append(("u", 2))

variables = OptvarHelper(vars)
x = variables.getVariable("x")
u = variables.getVariable("u")

w = np.array([1., 2., 3., 4., 5., 6.])
utest.assertTrue((x.getValue() == []).all())
utest.assertTrue((x.getValue(w) == w[0:4]).all())
utest.assertTrue((x.getValue() == w[0:4]).all())
utest.assertTrue((u.getValue(w) == w[4:]).all())

### SUBVARIABLES ###
vars = list()
vars.append(("q", 3))
vars.append(("qdot", 4))
variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")

x = np.array([1,2,3,4,5,6,7])
print(f"qdot.getValue(x): {qdot.getValue(x)}")
print(f"qdot[2:].getValue(): {qdot[2:].getValue()}")
utest.assertTrue(qdot[2:].getValue()[0] == qdot.getValue()[2])
utest.assertTrue(qdot[2:].getValue()[1] == qdot.getValue()[3])

x = np.array([8,9,10,11,12,13,14])
print(f"qdot[2:].getValue(x): {qdot[2:].getValue(x)}")
print(f"qdot.getValue(): {qdot.getValue()}")
utest.assertTrue(qdot[2:].getValue()[0] == qdot.getValue()[2])
utest.assertTrue(qdot[2:].getValue()[1] == qdot.getValue()[3])

### getId test ###
vars = list()
vars.append(("x", 4))
vars.append(("u", 2))
vars.append(("l", 1))
variables = OptvarHelper(vars)
x = variables.getVariable("x")
u = variables.getVariable("u")
l = variables.getVariable("l")
print(f"x.getM():\n {x.getM()}")
print(f"u.getM():\n {u.getM()}")
print(f"l.getM():\n {l.getM()}")
print(f"x.getId(): {x.getId()}")
print(f"u.getId(): {u.getId()}")
print(f"l.getId(): {l.getId()}")

A = np.array([[1., 2., 3., 4.],
              [5., 6., 7., 8.],
              [9., 10., 11., 12.],
              [13., 14., 15., 16.]])

B = np.array([[0., 0.],
              [10., 0.],
              [0., 10.],
              [0., 0.]])

C = np.array([[20.], [0.], [0.], [0.]])

print(f"A\n: {A}")
print(f"B\n: {B}")
print(f"C\n: {C}")

dx = A@x + B@u + C@l
print(f"dx:\n {dx}")
print(f"dx.getId(): {dx.getId()}")
utest.assertTrue(dx.getId() == 0)

A1 = dx.getM() @ x.getM().T
B1 = dx.getM() @ u.getM().T
C1 = dx.getM() @ l.getM().T
print(f"A1\n: {A1}")
print(f"B1\n: {B1}")
print(f"C1\n: {C1}")

utest.assertTrue((A1 == A).all())
utest.assertTrue((B1 == B).all())
utest.assertTrue((C1 == C).all())

A2 = dx.getM()[0:dx.getM().shape[0], x.getId():x.getId()+x.getM().shape[0]]
B2 = dx.getM()[0:dx.getM().shape[0], u.getId():u.getId()+u.getM().shape[0]]
C2 = dx.getM()[0:dx.getM().shape[0], l.getId():l.getId()+l.getM().shape[0]]
print(f"A2\n: {A2}")
print(f"B2\n: {B2}")
print(f"C2\n: {C2}")

utest.assertTrue((A1 == A2).all())
utest.assertTrue((B1 == B2).all())
utest.assertTrue((C1 == C2).all())

subx = x[2:3]
print(f"subx.getId(): {subx.getId()}")
utest.assertTrue(subx.getId() == 2)

subdx = x[1:3]
print(f"subdx.getId(): {subdx.getId()}")
utest.assertTrue(subdx.getId() == 1)

Q = np.random.rand(4,4)
R = np.random.rand(2,2)
L = np.random.rand(1,1)

H = np.block([[Q, np.zeros((4,2)), np.zeros((4,1))],
                [np.zeros((2,4)), R, np.zeros((2,1))],
                [np.zeros((1,4)), np.zeros((1,2)), L]])
print(f"H:\n {H}")
Q1 = x.getM() @ H @ x.getM().T
R1 = u.getM() @ H @ u.getM().T
L1 = l.getM() @ H @ l.getM().T
print(f"Q1:\n {Q1}")
print(f"R1:\n {R1}")
print(f"L1:\n {L1}")
utest.assertTrue((Q1 == Q).all())
utest.assertTrue((R1 == R).all())
utest.assertTrue((L1 == L).all())

Q2 = H[x.getId():x.getId()+x.getM().shape[0], x.getId():x.getId()+x.getM().shape[0]]
R2 = H[u.getId():u.getId()+u.getM().shape[0], u.getId():u.getId()+u.getM().shape[0]]
L2 = H[l.getId():l.getId()+l.getM().shape[0], l.getId():l.getId()+l.getM().shape[0]]
print(f"Q2:\n {Q2}")
print(f"R2:\n {R2}")
print(f"L2:\n {L2}")
utest.assertTrue((Q1 == Q2).all())
utest.assertTrue((R1 == R2).all())
utest.assertTrue((L1 == L2).all())