from libcpp.string cimport string
from libcpp.vector cimport vector
from libc.stdlib cimport free
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport fcl_defs as defs
import numpy as np
cimport numpy as np
ctypedef np.float64_t DOUBLE_t

cdef class Quaternion:
    cdef defs.Quaternion3f *thisptr
    def __cinit__(self, *args):
        import numbers
        if len(args) == 4:
            self.thisptr = new defs.Quaternion3f(<double?>args[0],
                                                 <double?>args[1],
                                                 <double?>args[2],
                                                 <double?>args[3])
        else:
            self.thisptr = new defs.Quaternion3f()
            if len(args) == 1 and isinstance(args[0], np.ndarray):
                self.thisptr.fromRotation(defs.Matrix3f(<double?>args[0][0, 0], <double?>args[0][0, 1], <double?>args[0][0, 2],
                                                        <double?>args[0][1, 0], <double?>args[0][1, 1], <double?>args[0][1, 2],
                                                        <double?>args[0][2, 0], <double?>args[0][2, 1], <double?>args[0][2, 2]))
            if len(args) == 2 and len(args[0]) == 3:
                self.thisptr.fromAxisAngle(defs.Vec3f(<double?>args[0][0],
                                                      <double?>args[0][1],
                                                      <double?>args[0][2]),
                                           <double?>args[1])
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

cdef class CollisionObject:
    cdef defs.CollisionObject *thisptr
    cdef defs.PyObject *geom
    def __cinit__(self, ShapeBase geom, tf=None):
        defs.Py_INCREF(<defs.PyObject*>geom)
        self.geom = <defs.PyObject*>geom
        self.thisptr = new defs.CollisionObject(defs.shared_ptr[defs.CollisionGeometry](<defs.CollisionGeometry*>geom.thisptr))
    def __dealloc__(self):
        if self.thisptr:
            free(self.thisptr)
        defs.Py_DECREF(self.geom)
    def getObjectType(self):
        return self.thisptr.getObjectType()
    def getNodeType(self):
        return self.thisptr.getNodeType()

cdef class ShapeBase:
    cdef defs.ShapeBase *thisptr
    def __cinit__(self):
        pass
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
    def getNodeType(self):
        if self.thisptr:
            return self.thisptr.getNodeType()
    def computeLocalAABB(self):
        if self.thisptr:
            self.thisptr.computeLocalAABB()

cdef class Box(ShapeBase):
    cdef object side
    def __cinit__(self, x, y, z):
        self.thisptr = new defs.Box(x, y, z)
    property side:
        def __get__(self):
            return [(<defs.Box*>self.thisptr).side[0],
                    (<defs.Box*>self.thisptr).side[1],
                    (<defs.Box*>self.thisptr).side[2]]
        def __set__(self, value):
            (<defs.Box*>self.thisptr).side[0] = <double?>value[0]
            (<defs.Box*>self.thisptr).side[1] = <double?>value[1]
            (<defs.Box*>self.thisptr).side[2] = <double?>value[2]

cdef class Sphere(ShapeBase):
    cdef object radius
    def __cinit__(self, radius):
        self.thisptr = new defs.Sphere(radius)
    property radius:
        def __get__(self):
            return (<defs.Sphere*>self.thisptr).radius
        def __set__(self, value):
            (<defs.Sphere*>self.thisptr).radius = <double?>value

cdef class Capsule(ShapeBase):
    cdef object radius
    cdef object lz
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
    cdef object radius
    cdef object lz
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
    cdef object radius
    cdef object lz
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
    def update(self):
        self.thisptr.update()
    def clear(self):
        self.thisptr.clear()
    def empty(self):
        return self.thisptr.empty()
    def size(self):
        return self.thisptr.size()

