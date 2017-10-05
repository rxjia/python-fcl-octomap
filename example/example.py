import numpy as np
import fcl

# Create simple geometries
box = fcl.Box(1.0, 2.0, 3.0)
sphere = fcl.Sphere(4.0)
cone = fcl.Cone(5.0, 6.0)
cyl = fcl.Cylinder(2.0, 2.0)

verts = np.array([[1.0, 1.0, 1.0],
                  [2.0, 1.0, 1.0],
                  [1.0, 2.0, 1.0],
                  [1.0, 1.0, 2.0]])
tris  = np.array([[0,2,1],
                  [0,3,2],
                  [0,1,3],
                  [1,2,3]])

# Create mesh geometry
m = fcl.BVHModel()
m.beginModel(len(verts), len(tris))
m.addSubModel(verts, tris)
m.endModel()

# Single object collision checking
req = fcl.CollisionRequest(enable_contact=True)
res = fcl.CollisionResult()
n_contacts = fcl.collide(fcl.CollisionObject(box, fcl.Transform()),
                         fcl.CollisionObject(cone, fcl.Transform()),
                         req, res)

print 'Collision between Box and Cone:'
print '-'*30
print 'Collision?: {}'.format(res.is_collision)
print 'Number of contacts: {}'.format(n_contacts)
print ''

req = fcl.CollisionRequest(enable_contact=True)
res = fcl.CollisionResult()
n_contacts = fcl.collide(fcl.CollisionObject(box, fcl.Transform()),
                         fcl.CollisionObject(cyl, fcl.Transform(np.array([6.0,0.0,0.0]))),
                         req, res)

print 'Collision between Box and Cylinder:'
print '-'*30
print 'Collision?: {}'.format(res.is_collision)
print 'Number of contacts: {}'.format(n_contacts)
print ''

req = fcl.CollisionRequest(enable_contact=True)
res = fcl.CollisionResult()
n_contacts = fcl.collide(fcl.CollisionObject(m, fcl.Transform(np.array([0.0,0.0,-1.0]))),
                         fcl.CollisionObject(cyl, fcl.Transform()),
                         req, res)

print 'Collision between Mesh and Cylinder:'
print '-'*30
print 'Collision?: {}'.format(res.is_collision)
print 'Number of contacts: {}'.format(n_contacts)
print ''


#objs = [fcl.CollisionObject(box),
#        fcl.CollisionObject(sphere),
#        fcl.CollisionObject(cone)]
#
## Register objects to DynamicAABBTreeCollisionManager
#manager = fcl.DynamicAABBTreeCollisionManager()
#print("Before register: ", manager.size())
#manager.registerObjects(objs)
#print("After register 1 : ", manager.size())
#manager.registerObject(fcl.CollisionObject(fcl.Cylinder(7.0, 8.0)))
#print("After register 2 : ", manager.size())
#
## Use Callback function
#def cb_func(obj1, obj2, res):
#    print("cb_func start")
#    res = collision_data.CollisionResult()
#    ret = fcl.collide(obj1, obj2, collision_data.CollisionRequest(), res)
#    ret = fcl.collide(obj1, obj2)
#    print("result: ", ret)
#    return ret
#res = collision_data.CollisionResult()
#manager.collide(res, cb_func)
#
## Collision calculation
#b = fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0))
#b.setTranslation(np.array([10.0, 0.0, 0.0]))
#s = fcl.CollisionObject(fcl.Sphere(4.0))
#s.setTranslation(np.array([-10.0, 0.0, 0.0]))
#result = collision_data.CollisionResult()
#ret = fcl.collide(b, s, collision_data.CollisionRequest(), result)
#
#print("-- Collision result: ", ret)
#for contact in result.contacts:
#    print(contact.o1)
#    print(contact.o2)
#for cost_source in result.cost_sources:
#    print(cost_source)
#
#b = fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0))
#b.setTranslation(np.array([10.0, 0.0, 0.0]))
#s = fcl.CollisionObject(fcl.Sphere(4.0))
#s.setTranslation(np.array([-10.0, 0.0, 0.0]))
#
#result = collision_data.DistanceResult()
#dis  = fcl.distance(b, s, collision_data.DistanceRequest(True), result)
#
#print("-- Distance result: ", dis)
#print(result.nearest_points)
