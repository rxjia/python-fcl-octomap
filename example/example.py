import numpy as np
import pdb
from fcl import fcl, collision_data

box = fcl.Box(1.0, 2.0, 3.0)
sphere = fcl.Sphere(4.0)
cone = fcl.Cone(5.0, 6.0)
objs = [fcl.CollisionObject(box),
        fcl.CollisionObject(sphere),
        fcl.CollisionObject(cone)]

# Register objects to DynamicAABBTreeCollisionManager
manager = fcl.DynamicAABBTreeCollisionManager()
print("Before register: ", manager.size())
manager.registerObjects(objs)
print("After register 1 : ", manager.size())
manager.registerObject(fcl.CollisionObject(fcl.Cylinder(7.0, 8.0)))
print("After register 2 : ", manager.size())

print box
print sphere
print cone

# Use Callback function
def cb_func(obj1, obj2, res):
    print("cb_func start")
    ret, res = fcl.collide(obj1, obj2, collision_data.CollisionRequest())
    print res.contacts[0].o1
    print res.contacts[0].o2
    print("result: ", ret)
    return ret
res = collision_data.CollisionResult()
manager.collide(res, cb_func)

# Collision calculation
b = fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0))
b.setTranslation(np.array([10.0, 0.0, 0.0]))
s = fcl.CollisionObject(fcl.Sphere(4.0))
s.setTranslation(np.array([-10.0, 0.0, 0.0]))
ret, result = fcl.collide(b, s, collision_data.CollisionRequest())

print("-- Collision result: ", ret)
for contact in result.contacts:
    print(contact.o1)
    print(contact.o2)
for cost_source in result.cost_sources:
    print(cost_source)

b = fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0))
b.setTranslation(np.array([10.0, 0.0, 0.0]))
s = fcl.CollisionObject(fcl.Sphere(4.0))
s.setTranslation(np.array([-10.0, 0.0, 0.0]))

dis, result = fcl.distance(b, s,
                           collision_data.DistanceRequest(True))

print("-- Distance result: ", dis)
print(result.nearest_points)
