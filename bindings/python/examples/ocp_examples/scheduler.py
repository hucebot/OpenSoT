from pyopensot.oc import *


sch = Scheduler()

sch.addContact("l", ["Left"])
sch.addContact("r", ["Right"])
sch.addContact("no_contacts", [])
# sch.addContact("r", ["Right"])

sch.addPhase(["l", "r"], 1.)
sch.addPhase(["no_contacts"], .4)
sch.addPhase(["l"], 1.)


seq = sch.getSequence(0.2, nodes_number = 20, current_time=0.0)

for n in seq:
    print(n)