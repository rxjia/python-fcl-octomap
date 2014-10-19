
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from libc.stdlib cimport free
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport fcl_defs as defs
import inspect
import transform as tf
from collision_data import *
cimport numpy as np
ctypedef np.float64_t DOUBLE_t

cdef class CollisionFunction:
    cdef:
        object py_func
        object py_args

    def __init__(self, py_func, py_args):
        self.py_func = py_func
        self.py_args = py_args
    cdef bool eval_func(self, defs.CollisionObject* o1, defs.CollisionObject* o2):
        cdef object py_r = defs.PyObject_CallObject(self.py_func,
                                                    (copy_ptr_collision_object(o1),
                                                     copy_ptr_collision_object(o2),
                                                     self.py_args))
        return <bool?>py_r

cdef class DistanceFunction:
    cdef:
        object py_func
        object py_args

    def __init__(self, py_func, py_args):
        self.py_func = py_func
        self.py_args = py_args

    cdef bool eval_func(self, defs.CollisionObject* o1, defs.CollisionObject* o2, defs.FCL_REAL& dist):
        cdef object py_r = defs.PyObject_CallObject(self.py_func,
                                                    (copy_ptr_collision_object(o1),
                                                     copy_ptr_collision_object(o2),
                                                     self.py_args))
        (&dist)[0] = <defs.FCL_REAL?>py_r[1]
        return <bool?>py_r[0]

cdef inline bool CollisionCallBack(defs.CollisionObject* o1, defs.CollisionObject* o2, void* cdata):
    return (<CollisionFunction>cdata).eval_func(o1, o2)

cdef inline bool DistanceCallBack(defs.CollisionObject* o1, defs.CollisionObject* o2, void* cdata, defs.FCL_REAL& dist):
    return (<DistanceFunction>cdata).eval_func(o1, o2, dist)

cdef vec3f_to_tuple(defs.Vec3f vec):
    return (vec[0], vec[1], vec[2])

cdef vec3f_to_list(defs.Vec3f vec):
    return [vec[0], vec[1], vec[2]]

cdef c_to_python_quaternion(defs.Quaternion3f q):
    return tf.Quaternion(q.getW(), q.getX(), q.getY(), q.getZ())

cdef class CollisionObject:
    cdef defs.CollisionObject *thisptr
    cdef defs.PyObject *geom
    cdef bool _no_instance
    def __cinit__(self, CollisionGeometry geom=CollisionGeometry(), tf=None, _no_instance=False):
        defs.Py_INCREF(<defs.PyObject*>geom)
        self.geom = <defs.PyObject*>geom
        self._no_instance = _no_instance
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
            if not self._no_instance:
                raise ValueError
    def __dealloc__(self):
        if self.thisptr and not self._no_instance:
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
        return c_to_python_quaternion(quat)
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

cdef class TriangleP(ShapeBase):
    def __cinit__(self, a, b, c):
        self.thisptr = new defs.TriangleP(defs.Vec3f(<double?>a[0], <double?>a[1], <double?>a[2]),
                                          defs.Vec3f(<double?>b[0], <double?>b[1], <double?>b[2]),
                                          defs.Vec3f(<double?>c[0], <double?>c[1], <double?>c[2]))
    property a:
        def __get__(self):
            return vec3f_to_tuple((<defs.TriangleP*>self.thisptr).a)
        def __set__(self, value):
            (<defs.TriangleP*>self.thisptr).a[0] = <double?>value[0]
            (<defs.TriangleP*>self.thisptr).a[1] = <double?>value[1]
            (<defs.TriangleP*>self.thisptr).a[2] = <double?>value[2]
    property b:
        def __get__(self):
            return vec3f_to_tuple((<defs.TriangleP*>self.thisptr).b)
        def __set__(self, value):
            (<defs.TriangleP*>self.thisptr).b[0] = <double?>value[0]
            (<defs.TriangleP*>self.thisptr).b[1] = <double?>value[1]
            (<defs.TriangleP*>self.thisptr).b[2] = <double?>value[2]
    property c:
        def __get__(self):
            return vec3f_to_tuple((<defs.TriangleP*>self.thisptr).c)
        def __set__(self, value):
            (<defs.TriangleP*>self.thisptr).c[0] = <double?>value[0]
            (<defs.TriangleP*>self.thisptr).c[1] = <double?>value[1]
            (<defs.TriangleP*>self.thisptr).c[2] = <double?>value[2]

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
    cdef vector[defs.PyObject*]* objs
    def __cinit__(self):
        self.thisptr = new defs.DynamicAABBTreeCollisionManager()
        self.objs = new vector[defs.PyObject*   ]()
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
        for idx in range(self.objs.size()):
            defs.Py_DECREF(deref(self.objs)[idx])
    def registerObjects(self, other_objs):
        cdef vector[defs.CollisionObject*] pobjs
        for o in other_objs:
            defs.Py_INCREF(<defs.PyObject*>o)
            self.objs.push_back(<defs.PyObject*>o)
            pobjs.push_back((<CollisionObject?>o).thisptr)
        self.thisptr.registerObjects(pobjs)
    def registerObject(self, CollisionObject obj):
        defs.Py_INCREF(<defs.PyObject*>obj)
        self.objs.push_back(<defs.PyObject*>obj)
        self.thisptr.registerObject(obj.thisptr)
    def collide(self, *args):
        if len(args) == 2 and inspect.isfunction(args[1]):
            fn = CollisionFunction(args[1], args[0])
            self.thisptr.collide(<void*>fn, CollisionCallBack)
        elif len(args) == 3 and inspect.isfunction(args[2]):
            fn = CollisionFunction(args[2], args[1])
            self.thisptr.collide((<CollisionObject?>args[0]).thisptr, <void*>fn, CollisionCallBack)
        else:
            raise ValueError

    def distance(self, *args):
        if len(args) == 2 and inspect.isfunction(args[1]):
            fn = DistanceFunction(args[1], args[0])
            self.thisptr.distance(<void*> fn, DistanceCallBack)
        elif len(args) == 3 and inspect.isfunction(args[2]):
            fn = DistanceFunction(args[2], args[1])
            self.thisptr.distance((<CollisionObject?> args[0]).thisptr, <void*> fn, DistanceCallBack)
        else:
            raise ValueError

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
        obj = TriangleP(np.zeros(3), np.zeros(3), np.zeros(3))
        memcpy(obj.thisptr, geom, sizeof(defs.TriangleP))
        return obj
    elif geom.getNodeType() == defs.GEOM_HALFSPACE:
        obj = Halfspace(np.zeros(3), 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Halfspace))
        return obj
    elif geom.getNodeType() == defs.GEOM_PLANE:
        obj = Plane(np.zeros(3), 0)
        memcpy(obj.thisptr, geom, sizeof(defs.Plane))
        return obj

cdef copy_ptr_collision_object(defs.CollisionObject* cobj):
    co = CollisionObject(_no_instance=True)
    (<CollisionObject>co).thisptr = cobj
    return co

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

def distance(CollisionObject o1, CollisionObject o2, request):
    cdef defs.DistanceResult result
    cdef double dis = defs.distance(o1.thisptr,
                                    o2.thisptr,
                                    defs.DistanceRequest(<bool?>request.enable_nearest_points),
                                    result)
    dis_res = DistanceResult()
    dis_res.min_distance = result.min_distance
    dis_res.nearest_points = [vec3f_to_tuple(result.nearest_points[0]),
                              vec3f_to_tuple(result.nearest_points[1])]
    dis_res.o1 = c_to_python_collision_geometry(result.o1)
    dis_res.o2 = c_to_python_collision_geometry(result.o2)
    dis_res.b1 = result.b1
    dis_res.b2 = result.b2
    return dis, dis_res
