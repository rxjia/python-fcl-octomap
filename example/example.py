import numpy as np
import fcl

def print_collision_result(o1_name, o2_name, result):
    print 'Collision between {} and {}:'.format(o1_name, o2_name)
    print '-'*30
    print 'Collision?: {}'.format(result.is_collision)
    print 'Number of contacts: {}'.format(len(result.contacts))
    print ''

def print_continuous_collision_result(o1_name, o2_name, result):
    print 'Continuous collision between {} and {}:'.format(o1_name, o2_name)
    print '-'*30
    print 'Collision?: {}'.format(result.is_collide)
    print 'Time of collision: {}'.format(result.time_of_contact)
    print ''

def print_distance_result(o1_name, o2_name, result):
    print 'Distance between {} and {}:'.format(o1_name, o2_name)
    print '-'*30
    print 'Distance: {}'.format(result.min_distance)
    print 'Closest Points:'
    print result.nearest_points[0]
    print result.nearest_points[1]
    print ''

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
mesh = fcl.BVHModel()
mesh.beginModel(len(verts), len(tris))
mesh.addSubModel(verts, tris)
mesh.endModel()

#=====================================================================
# Pairwise collision checking
#=====================================================================
print '='*60
print 'Testing Pairwise Collision Checking'
print '='*60
print ''

req = fcl.CollisionRequest(enable_contact=True)
res = fcl.CollisionResult()

n_contacts = fcl.collide(fcl.CollisionObject(box, fcl.Transform()),
                         fcl.CollisionObject(cone, fcl.Transform()),
                         req, res)
print_collision_result('Box', 'Cone', res)

n_contacts = fcl.collide(fcl.CollisionObject(box, fcl.Transform()),
                         fcl.CollisionObject(cyl, fcl.Transform(np.array([6.0,0.0,0.0]))),
                         req, res)
print_collision_result('Box', 'Cylinder', res)

n_contacts = fcl.collide(fcl.CollisionObject(mesh, fcl.Transform(np.array([0.0,0.0,-1.0]))),
                         fcl.CollisionObject(cyl, fcl.Transform()),
                         req, res)
print_collision_result('Box', 'Mesh', res)

#=====================================================================
# Pairwise distance compuatation
#=====================================================================
print '='*60
print 'Testing Pairwise Distance Checking'
print '='*60
print ''

req = fcl.DistanceRequest(enable_nearest_points=True)
res = fcl.DistanceResult()

dist = fcl.distance(fcl.CollisionObject(box, fcl.Transform()),
                    fcl.CollisionObject(cone, fcl.Transform()),
                    req, res)
print_distance_result('Box', 'Cone', res)

dist = fcl.distance(fcl.CollisionObject(box, fcl.Transform()),
                    fcl.CollisionObject(cyl, fcl.Transform(np.array([6.0,0.0,0.0]))),
                    req, res)
print_distance_result('Box', 'Cylinder', res)

dist = fcl.distance(fcl.CollisionObject(box, fcl.Transform()),
                    fcl.CollisionObject(box, fcl.Transform(np.array([1.01,0.0,0.0]))),
                    req, res)
print_distance_result('Box', 'Box', res)

#=====================================================================
# Pairwise continuous collision checking
#=====================================================================
print '='*60
print 'Testing Pairwise Continuous Collision Checking'
print '='*60
print ''

req = fcl.ContinuousCollisionRequest()
res = fcl.ContinuousCollisionResult()

dist = fcl.continuousCollide(fcl.CollisionObject(box, fcl.Transform(np.array([20.0, 0.0, 0.0]))),
                             fcl.Transform(),
                             fcl.CollisionObject(cyl, fcl.Transform()),
                             fcl.Transform(np.array([3.0, 0.0, 0.0])),
                             req, res)
print_continuous_collision_result('Box', 'Cylinder', res)

#=====================================================================
# Managed one-to-many collision checking
#=====================================================================
print '='*60
print 'Testing Managed Collision Checking'
print '='*60
print ''
objs1 = [fcl.CollisionObject(box, fcl.Transform(np.array([20,0,0]))), fcl.CollisionObject(sphere)]
objs2 = [fcl.CollisionObject(cone), fcl.CollisionObject(mesh)]

manager1 = fcl.DynamicAABBTreeCollisionManager()
manager2 = fcl.DynamicAABBTreeCollisionManager()

manager1.registerObjects(objs1)
manager2.registerObjects(objs2)

cdata = fcl.CollisionData()
manager1.collide(cdata, fcl.defaultCollisionCallback)
print cdata.result.is_collision

cdata = fcl.DistanceData()
manager1.distance(cdata, fcl.defaultDistanceCallback)
print cdata.result.min_distance
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
