import numpy as np

import fcl


x = np.random.random([100, 3])

object1 = fcl.OcTree(r=0.02, points=x)

object2 = fcl.Box(1, 2, 3)
o1 = fcl.CollisionObject(object1, fcl.Transform())
o2 = fcl.CollisionObject(object2, fcl.Transform())

request = fcl.CollisionRequest()
result = fcl.CollisionResult()

req = fcl.collide(o2, o2, request, result)

print(req)