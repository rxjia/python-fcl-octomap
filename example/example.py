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

ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
                                              fcl.Transform(fcl.Quaternion(), [10.0, 0.0, 0.0])),
                          fcl.CollisionObject(fcl.Sphere(4.0),
                                              fcl.Transform(fcl.Quaternion(), [-10.0, 0.0, 0.0])),
                          fcl.CollisionRequest())
print ret
for contact in result.contacts:
    print contact.o1
    print contact.o2
for cost_source in result.cost_sources:
    print cost_source

ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
                                              fcl.Transform(fcl.Quaternion())),
                          fcl.CollisionObject(fcl.Sphere(4.0),
                                              fcl.Transform(fcl.Quaternion(), [0.0, 0.0, 0.0])),
                          fcl.CollisionRequest())
print ret
for contact in result.contacts:
    print contact.o1
    print contact.o2
for cost_source in result.cost_sources:
    print cost_source
