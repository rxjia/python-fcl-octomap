import fcl

objs = [fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0)),
        fcl.CollisionObject(fcl.Sphere(4.0)),
        fcl.CollisionObject(fcl.Cone(5.0, 6.0))]

manager = fcl.DynamicAABBTreeCollisionManager()
print "Before resgister: ", manager.size()
manager.registerObjects(objs)
print "After register 1 : ", manager.size()
manager.registerObject(fcl.CollisionObject(fcl.Cylinder(7.0, 8.0)))
print "After register 2 : ", manager.size()

ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0)),
                           fcl.CollisionObject(fcl.Sphere(4.0)),
                           fcl.CollisionRequest())
print ret, result.contacts
