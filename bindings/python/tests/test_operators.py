from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, MinimumEffort
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
import pyopensot as pysot
import numpy as np
import unittest

class TestOperators(unittest.TestCase):
    def test_operators(self):
        resource = "panda.urdf";
        urdf_path = pysot.find(resource)
        print(f"Loading {urdf_path}")
        urdf_string = pysot.ReadFile(urdf_path)

        model = xbi.ModelInterface2(urdf_string)

        q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
        model.setJointPosition(q)
        model.update()

        C = Cartesian("Cartesian", model, "fp3_link8", "world")
        Cp = C%[0,1,2]
        Co = C%[3,4,5]
        CC = Cp + Co

        for cc, c in zip(C.getA(), CC.getA()):
            self.assertEqual(cc.tolist(), c.tolist())
        self.assertEqual(C.getb().tolist(), CC.getb().tolist())

        p = Postural(model)

        D = C + p%[0]
        for i in range(0, 6):
            self.assertEqual(D.getA()[i,:].tolist(), C.getA()[i,:].tolist())
        self.assertEqual(D.getA()[6,:].tolist(), p.getA()[0,:].tolist())
        self.assertEqual(D.getb()[0:6].tolist(), C.getb().tolist())
        self.assertEqual(D.getb()[6], p.getb()[0])

        C2 = Cartesian("Cartesian2", model, "fp3_link5", "world")
        C3 = Cartesian("Cartesian3", model, "fp3_link3", "world")

        A1 = C + C2 + C3
        A2 = A1 + p

        for i in range(0, 6):
            self.assertEqual(A2.getA()[i,:].tolist(), C.getA()[i,:].tolist())
        self.assertEqual(A2.getb()[0:6].tolist(), C.getb().tolist())
        for i in range(6, 12):
            self.assertEqual(A2.getA()[i,:].tolist(), C2.getA()[i-6,:].tolist())
        self.assertEqual(A2.getb()[6:12].tolist(), C2.getb().tolist())
        for i in range(12, 18):
            self.assertEqual(A2.getA()[i,:].tolist(), C3.getA()[i-12,:].tolist())
        self.assertEqual(A2.getb()[12:18].tolist(), C3.getb().tolist())
        for i in range(18, 25):
            self.assertEqual(A2.getA()[i,:].tolist(), p.getA()[i-18,:].tolist())
        self.assertEqual(A2.getb()[18:25].tolist(), p.getb().tolist())

        T = 2*C
        for i in range(0, 6):
            self.assertEqual(T.getWeight()[i,:].tolist(), C.getWeight()[i,:].tolist())

        W = C.getWeight()
        W = 3*W
        T = pysot.mul(W,C)
        for i in range(0, 6):
            self.assertEqual(T.getWeight()[i,:].tolist(), C.getWeight()[i,:].tolist())

        qmin, qmax = model.getJointLimits()
        qlims = JointLimits(model, qmax, qmin)
        ql = qlims%[0,2,5]
        print(ql.getAineq())
        self.assertEqual(ql.getbLowerBound()[0], qlims.getLowerBound()[0])
        self.assertEqual(ql.getbLowerBound()[1], qlims.getLowerBound()[2])
        self.assertEqual(ql.getbLowerBound()[2], qlims.getLowerBound()[5])
        self.assertEqual(ql.getbUpperBound()[0], qlims.getUpperBound()[0])
        self.assertEqual(ql.getbUpperBound()[1], qlims.getUpperBound()[2])
        self.assertEqual(ql.getbUpperBound()[2], qlims.getUpperBound()[5])

        S = C/C2
        self.assertEqual(S.getStack()[0].getTaskID(), C.getTaskID())
        self.assertEqual(S.getStack()[1].getTaskID(), C2.getTaskID())

        S1 = (C2 + C3) / p
        self.assertEqual(S1.getStack()[0].getTaskID(), (C2 + C3).getTaskID())
        self.assertEqual(S1.getStack()[1].getTaskID(), p.getTaskID())

        S = S / p
        self.assertEqual(S.getStack()[0].getTaskID(), C.getTaskID())
        self.assertEqual(S.getStack()[1].getTaskID(), C2.getTaskID())
        self.assertEqual(S.getStack()[2].getTaskID(), p.getTaskID())

        S2 = (C+C3) / S1
        for t in S2.getStack():
            print(t.getTaskID())
        self.assertEqual(S2.getStack()[0].getTaskID(), (C+C3).getTaskID())
        self.assertEqual(S2.getStack()[1].getTaskID(), (C2+C3).getTaskID())
        self.assertEqual(S2.getStack()[2].getTaskID(), p.getTaskID())

        SS = C/C2/C3
        self.assertEqual(SS.getStack()[0].getTaskID(), C.getTaskID())
        self.assertEqual(SS.getStack()[1].getTaskID(), C2.getTaskID())
        self.assertEqual(SS.getStack()[2].getTaskID(), C3.getTaskID())

        SSS = pysot.hard(SS, S1)
        self.assertEqual(SSS.getStack()[0].getTaskID(), C.getTaskID())
        self.assertEqual(SSS.getStack()[1].getTaskID(), C2.getTaskID())
        self.assertEqual(SSS.getStack()[2].getTaskID(), C3.getTaskID())
        self.assertEqual(SSS.getStack()[3].getTaskID(), (C2+C3).getTaskID())
        self.assertEqual(SSS.getStack()[4].getTaskID(), p.getTaskID())


        TC = C<<qlims
        self.assertEqual(TC.getConstraints()[0].getConstraintID(), qlims.getConstraintID())

        dqmax = model.getVelocityLimits()
        dt = 1./100.
        dqlims = VelocityLimits(model, dqmax, dt)

        TCC = (C2+C3)<<qlims<<dqlims
        self.assertEqual(TCC.getConstraints()[0].getConstraintID(), qlims.getConstraintID())
        self.assertEqual(TCC.getConstraints()[1].getConstraintID(), dqlims.getConstraintID())

        TTC = C3<<C2
        self.assertEqual(TTC.getTaskID(), C3.getTaskID())
        self.assertEqual(TTC.getConstraints()[0].getConstraintID(), C2.getTaskID())

        S1C = S1<<qlims
        self.assertEqual(S1C.getBoundsList()[0].getConstraintID(), qlims.getConstraintID())

        S2C = (C2/C3)<<dqlims<<qlims
        self.assertEqual(S2C.getBoundsList()[0].getConstraintID(), dqlims.getConstraintID())
        self.assertEqual(S2C.getBoundsList()[1].getConstraintID(), qlims.getConstraintID())

        S23 = (C2/C3)<<p
        self.assertEqual(S23.getBoundsList()[0].getConstraintID(), p.getTaskID())

        S4 = (C/C2)/(C3/p)
        S11 = C/C2
        S12 = C3/p
        S14 = S11/S12
        for i in range(0,4):
            self.assertEqual(S4.getStack()[i].getTaskID(), S14.getStack()[i].getTaskID())

if __name__ == "__main__":
    unittest.main()
