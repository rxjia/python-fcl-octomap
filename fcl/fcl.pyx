
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from libc.stdlib cimport free
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport fcl_defs as defs
import numpy as np
import transform as tf
from collision_data import *
cimport numpy as np
ctypedef np.float64_t DOUBLE_t



cdef vec3f_to_tuple(defs.Vec3f vec):
    return (vec[0], vec[1], vec[2])

cdef vec3f_to_list(defs.Vec3f vec):
    return [vec[0], vec[1], vec[2]]

cdef class CollisionObject:
    cdef defs.CollisionObject *thisptr
    cdef defs.PyObject *geom
    def __cinit__(self, CollisionGeometry geom=CollisionGeometry(), tf=None):
        defs.Py_INCREF(<defs.PyObject*>geom)
        self.geom = <defs.PyObject*>geom
        if not geom.getNodeType() is None:
            if not tf is None:
                self.thisptr = new defs.CollisionObject(defs.shared_ptr[defs.CollisionGeometry](geom.thisptr),
                                                        defs.Transform3f(defs.Quaternion3f(<double?>tf.q.w,
                                                                                           <double?>tf.q.x,
                                                                                           <double?>tf.q.y,
                                                                                           <double?>tf.q.z),
                                                                         defs.Vec3f(<double?>tf.t[0],
                                                                                    <double?>tf.t[1],
                                                                                    <double?>tf.t[2])))
            else:
                self.thisptr = new defs.CollisionObject(defs.shared_ptr[defs.CollisionGeometry](geom.thisptr))
        else:
            raise ValueError
    def __dealloc__(self):
        if self.thisptr:
            free(self.thisptr)
        defs.Py_DECREF(self.geom)
    def getObjectType(self):
        return self.thisptr.getObjectType()
    def getNodeType(self):
        return self.thisptr.getNodeType()
    def getTranslation(self):
        return vec3f_to_tuple(self.thisptr.getTranslation())
    def getQuatRotation(self):
        cdef defs.Quaternion3f quat = self.thisptr.getQuatRotation()
        return tf.Quaternion(quat.getW(), quat.getX(), quat.getY(), quat.getZ())
    def setTranslation(self, vec):
        self.thisptr.setTranslation(defs.Vec3f(<double?>vec[0], <double?>vec[1], <double?>vec[2]))
    def setQuatRotation(self, q):
        self.thisptr.setQuatRotation(defs.Quaternion3f(<double?>q[0], <double?>q[1], <double?>q[2], <double?>q[3]))
    def setTransform(self, q, vec):
        self.thisptr.setTransform(defs.Quaternion3f(<double?>q[0], <double?>q[1], <double?>q[2], <double?>q[3]),
                                 defs.Vec3f(<double?>vec[0], <double?>vec[1], <double?>vec[2]))
    def isOccupied(self):
        return self.thisptr.isOccupied()
    def isFree(self):
        return self.thisptr.isFree()
    def isUncertain(self):
        return self.thisptr.isUncertain()

cdef class CollisionGeometry:
    cdef defs.CollisionGeometry *thisptr
    def __cinit__(self):
        pass
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
    def getNodeType(self):
        if self.thisptr:
            return self.thisptr.getNodeType()
        else:
            return None
    def computeLocalAABB(self):
        if self.thisptr:
            self.thisptr.computeLocalAABB()
        else:
            return None
    property aabb_center:
        def __get__(self):
            if self.thisptr:
                return vec3f_to_tuple(self.thisptr.aabb_center)
            else:
                return None
        def __set__(self, value):
            if self.thisptr:
                self.thisptr.aabb_center[0] = value[0]
                self.thisptr.aabb_center[1] = value[1]
                self.thisptr.aabb_center[2] = value[2]
            else:
                raise ReferenceError

cdef class ShapeBase(CollisionGeometry):
    def __cinit__(self):
        pass

cdef class Triangle2(ShapeBase):
    def __cinit__(self, a, b, c):
        self.thisptr = new defs.Triangle2(defs.Vec3f(<double?>a[0], <double?>a[1], <double?>a[2]),
                                          defs.Vec3f(<double?>b[0], <double?>b[1], <double?>b[2]),
                                          defs.Vec3f(<double?>c[0], <double?>c[1], <double?>c[2]))
    property a:
        def __get__(self):
            return vec3f_to_tuple((<defs.Triangle2*>self.thisptr).a)
        def __set__(self, value):
            (<defs.Triangle2*>self.thisptr).a[0] = <double?>value[0]
            (<defs.Triangle2*>self.thisptr).a[1] = <double?>value[1]
            (<defs.Triangle2*>self.thisptr).a[2] = <double?>value[2]
    property b:
        def __get__(self):
            return vec3f_to_tuple((<defs.Triangle2*>self.thisptr).b)
        def __set__(self, value):
            (<defs.Triangle2*>self.thisptr).b[0] = <double?>value[0]
            (<defs.Triangle2*>self.thisptr).b[1] = <double?>value[1]
            (<defs.Triangle2*>self.thisptr).b[2] = <double?>value[2]
    property c:
        def __get__(self):
            return vec3f_to_tuple((<defs.Triangle2*>self.thisptr).c)
        def __set__(self, value):
            (<defs.Triangle2*>self.thisptr).c[0] = <double?>value[0]
            (<defs.Triangle2*>self.thisptr).c[1] = <double?>value[1]
            (<defs.Triangle2*>self.thisptr).c[2] = <double?>value[2]

cdef class Box(ShapeBase):
    def __cinit__(self, x, y, z):
        self.thisptr = new defs.Box(x, y, z)
    property side:
        def __get__(self):
            return vec3f_to_tuple((<defs.Box*>self.thisptr).side)
        def __set__(self, value):
            (<defs.Box*>self.thisptr).side[0] = <double?>value[0]
            (<defs.Box*>self.thisptr).side[1] = <double?>value[1]
            (<defs.Box*>self.thisptr).side[2] = <double?>value[2]

cdef class Sphere(ShapeBase):
    def __cinit__(self, radius):
        self.thisptr = new defs.Sphere(radius)
    property radius:
        def __get__(self):
            return (<defs.Sphere*>self.thisptr).radius
        def __set__(self, value):
            (<defs.Sphere*>self.thisptr).radius = <double?>value

cdef class Capsule(ShapeBase):
    def __cinit__(self, radius, lz):
        self.thisptr = new defs.Capsule(radius, lz)
    property radius:
        def __get__(self):
            return (<defs.Capsule*>self.thisptr).radius
        def __set__(self, value):
            (<defs.Capsule*>self.thisptr).radius = <double?>value
    property lz:
        def __get__(self):
            return (<defs.Capsule*>self.thisptr).lz
        def __set__(self, value):
            (<defs.Capsule*>self.thisptr).lz = <double?>value

cdef class Cone(ShapeBase):
    def __cinit__(self, radius, lz):
        self.thisptr = new defs.Cone(radius, lz)
    property radius:
        def __get__(self):
            return (<defs.Cone*>self.thisptr).radius
        def __set__(self, value):
            (<defs.Cone*>self.thisptr).radius = <double?>value
    property lz:
        def __get__(self):
            return (<defs.Cone*>self.thisptr).lz
        def __set__(self, value):
            (<defs.Cone*>self.thisptr).lz = <double?>value

cdef class Cylinder(ShapeBase):
    def __cinit__(self, radius, lz):
        self.thisptr = new defs.Cylinder(radius, lz)
    property radius:
        def __get__(self):
            return (<defs.Cylinder*>self.thisptr).radius
        def __set__(self, value):
            (<defs.Cylinder*>self.thisptr).radius = <double?>value
    property lz:
        def __get__(self):
            return (<defs.Cylinder*>self.thisptr).lz
        def __set__(self, value):
            (<defs.Cylinder*>self.thisptr).lz = <double?>value

cdef class Halfspace(ShapeBase):
    def __cinit__(self, n, d):
        self.thisptr = new defs.Halfspace(defs.Vec3f(<double?>n[0],
                                                     <double?>n[1],
                                                     <double?>n[2]),
                                          <double?>d)
    property n:
        def __get__(self):
            return vec3f_to_tuple((<defs.Halfspace*>self.thisptr).n)
        def __set__(self, value):
            (<defs.Halfspace*>self.thisptr).n[0] = <double?>value[0]
            (<defs.Halfspace*>self.thisptr).n[1] = <double?>value[1]
            (<defs.Halfspace*>self.thisptr).n[2] = <double?>value[2]
    property d:
        def __get__(self):
            return (<defs.Halfspace*>self.thisptr).d
        def __set__(self, value):
            (<defs.Halfspace*>self.thisptr).d = <double?>value

cdef class Plane(ShapeBase):
    def __cinit__(self, n, d):
        self.thisptr = new defs.Plane(defs.Vec3f(<double?>n[0],
                                                 <double?>n[1],
                                                 <double?>n[2]),
                                      <double?>d)
    property n:
        def __get__(self):
            return vec3f_to_tuple((<defs.Plane*>self.thisptr).n)
        def __set__(self, value):
            (<defs.Plane*>self.thisptr).n[0] = <double?>value[0]
            (<defs.Plane*>self.thisptr).n[1] = <double?>value[1]
            (<defs.Plane*>self.thisptr).n[2] = <double?>value[2]
    property d:
        def __get__(self):
            return (<defs.Plane*>self.thisptr).d
        def __set__(self, value):
            (<defs.Plane*>self.thisptr).d = <double?>value

class Contact:
    def __init__(self):
        self.o1 = CollisionGeometry()
        self.o2 = CollisionGeometry()
        self.b1 = 0
        self.b2 = 0
        self.normal = [0.0, 0.0, 0.0]
        self.pos = [0.0, 0.0, 0.0]
        self.penetration_depth = 0.0

cdef class DynamicAABBTreeCollisionManager:
    cdef defs.DynamicAABBTreeCollisionManager *thisptr
    def __cinit__(self):
        self.thisptr = new defs.DynamicAABBTreeCollisionManager()
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
    def registerObjects(self, other_objs):
        cdef vector[defs.CollisionObject*] pobjs
        for o in other_objs:
            pobjs.push_back((<CollisionObject?>o).thisptr)
        self.thisptr.registerObjects(pobjs)
    def registerObject(self, CollisionObject obj):
        self.thisptr.registerObject(obj.thisptr)
    def setup(self):
        self.thisptr.setup()
    def update(self, arg=None):
        cdef vector[defs.CollisionObject*] objs
        if hasattr(arg, "__len__"):
            for a in arg:
                objs.push_back((<CollisionObject>a).thisptr)
            self.thisptr.update(objs)
        elif arg is None:
            self.thisptr.update()    
        else:
            self.thisptr.update((<CollisionObject>arg).thisptr)
    def clear(self):
        self.thisptr.clear()
    def empty(self):
        return self.thisptr.empty()
    def size(self):
        return self.thisptr.size()
    property max_tree_nonbalanced_level:
        def __get__(self):
            return self.thisptr.max_tree_nonbalanced_level
        def __set__(self, value):
            self.thisptr.max_tree_nonbalanced_level = <int?>value
    property tree_incremental_balance_pass:
        def __get__(self):
            return self.thisptr.tree_incremental_balance_pass
        def __set__(self, value):
            self.thisptr.tree_incremental_balance_pass = <int?>value
    property tree_topdown_balance_threshold:
        def __get__(self):
            return self.thisptr.tree_topdown_balance_threshold
        def __set__(self, value):
            self.thisptr.tree_topdown_balance_threshold = <int?>value
    property tree_topdown_level:
        def __get__(self):
            return self.thisptr.tree_topdown_level
        def __set__(self, value):
            self.thisptr.tree_topdown_level = <int?>value
    property tree_init_level:
        def __get__(self):
            return self.thisptr.tree_init_level
        def __set__(self, value):
            self.thisptr.tree_init_level = <int?>value
    property octree_as_geometry_collide:
        def __get__(self):
            return self.thisptr.octree_as_geometry_collide
        def __set__(self, value):
            self.thisptr.octree_as_geometry_collide = <bool?>value
    property octree_as_geometry_distance:
        def __get__(self):
            return self.thisptr.octree_as_geometry_distance
        def __set__(self, value):
            self.thisptr.octree_as_geometry_distance = <bool?>value

cdef c_to_python_collision_geometry(defs.const_CollisionGeometry* geom):
    if geom.getNodeType() == defs.GEOM_BOX:
        obj = Box(0, 0, 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Box))
        return obj
    elif geom.getNodeType() == defs.GEOM_SPHERE:
        obj = Sphere(0)
        memcpy(obj.thisptr, geom, sizeof(defs.Sphere))
        return obj
    elif geom.getNodeType() == defs.GEOM_CAPSULE:
        obj = Capsule(0, 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Capsule))
        return obj
    elif geom.getNodeType() == defs.GEOM_CONE:
        obj = Cone(0, 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Cone))
        return obj
    elif geom.getNodeType() == defs.GEOM_CYLINDER:
        obj = Cylinder(0, 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Cylinder))
        return obj
    elif geom.getNodeType() == defs.GEOM_TRIANGLE:
        obj = Triangle2(np.zeros(3), np.zeros(3), np.zeros(3))
        memcpy(obj.thisptr, geom, sizeof(defs.Triangle2))
        return obj
    elif geom.getNodeType() == defs.GEOM_HALFSPACE:
        obj = Halfspace(np.zeros(3), 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Halfspace))
        return obj
    elif geom.getNodeType() == defs.GEOM_PLANE:
        obj = Plane(np.zeros(3), 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Plane))
        return obj

cdef c_to_python_contact(defs.Contact contact):
    c = Contact()
    c.o1 = c_to_python_collision_geometry(contact.o1)
    c.o2 = c_to_python_collision_geometry(contact.o2)
    c.b1 = contact.b1
    c.b2 = contact.b2
    c.normal = vec3f_to_list(contact.normal)
    c.pos = vec3f_to_list(contact.pos)
    c.penetration_depth = contact.penetration_depth
    return c

cdef c_to_python_costsource(defs.CostSource cost_source):
    c = CostSource()
    c.aabb_min = vec3f_to_list(cost_source.aabb_min)
    c.aabb_max = vec3f_to_list(cost_source.aabb_max)
    c.cost_density = cost_source.cost_density
    c.total_cost = cost_source.total_cost
    return c

def collide(CollisionObject o1, CollisionObject o2, request):
    cdef defs.CollisionResult result
    cdef size_t ret = defs.collide(o1.thisptr,
                                   o2.thisptr,
                                   defs.CollisionRequest(<size_t?>request.num_max_contacts,
                                                         <bool?>request.enable_contact,
                                                         <size_t?>request.num_max_cost_sources,
                                                         <bool>request.enable_cost,
                                                         <bool>request.use_approximate_cost),
                                   result)
    col_res = CollisionResult()
    cdef vector[defs.Contact] contacts
    result.getContacts(contacts)
    cdef vector[defs.CostSource] costs
    result.getCostSources(costs)
    for idx in range(contacts.size()):
        col_res.contacts.append(c_to_python_contact(contacts[idx]))
    for idx in range(costs.size()):
        col_res.cost_sources.append(c_to_python_costsource(costs[idx]))
    return ret, col_res
