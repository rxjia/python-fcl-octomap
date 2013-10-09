from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from libc.stdlib cimport free
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport fcl_defs as defs
import numpy as np
cimport numpy as np
ctypedef np.float64_t DOUBLE_t

class OBJECT_TYPE:
    OT_UNKNOWN, OT_BVH, OT_GEOM, OT_OCTREE, OT_COUNT = range(5)
    def __init__(self):
        pass

class NODE_TYPE:
    BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24,\
    GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE,\
    GEOM_HALFSPACE, GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT = range(20)
    def __init__(self):
        pass

def rotation_to_quaternion(rot):
    q = np.array(4)
    q[0] = np.sqrt(max((rot[0, 0] + rot[1, 1] + rot[2, 2] + 1.0)/4.0, 0.0))
    q[1] = np.sqrt(max((rot[0, 0] - rot[1, 1] - rot[2, 2] + 1.0)/4.0, 0.0))
    q[2] = np.sqrt(max((-rot[0, 0] + rot[1, 1] - rot[2, 2] + 1.0)/4.0, 0.0))
    q[3] = np.sqrt(max((-rot[0, 0] - rot[1, 1] + rot[2, 2] + 1.0)/4.0, 0.0))
    if q[0] >= q[1] and q[0] >= q[2] and q[0] >= q[3]:
        q[1] *= np.sign(rot[2, 1] - rot[1, 2])
        q[2] *= np.sign(rot[0, 2] - rot[2, 0])
        q[3] *= np.sign(rot[1, 0] - rot[0, 1])
    elif q[1] >= q[0] and q[1] >= q[2] and q[1] >= q[3]:
        q[0] *= np.sign(rot[2, 1] - rot[1, 2])
        q[2] *= np.sign(rot[1, 0] + rot[0, 1])
        q[3] *= np.sign(rot[0, 2] + rot[2, 0])
    elif q[2] >= q[0] and q[2] >= q[1] and q[2] >= q[3]:
        q[0] *= np.sign(rot[0, 2] - rot[2, 0])
        q[1] *= np.sign(rot[1, 0] + rot[0, 1])
        q[3] *= np.sign(rot[2, 1] + rot[1, 2])
    elif q[3] >= q[0] and q[3] >= q[1] and q[3] >= q[2]:
        q[0] *= np.sign(rot[1, 0] - rot[0, 1])
        q[1] *= np.sign(rot[2, 0] + rot[0, 2])
        q[2] *= np.sign(rot[2, 1] + rot[1, 2])
    else:
        raise ValueError
    return q/np.linalg.norm(q)

class Quaternion:
    def __init__(self, *args):
        if len(args) == 0:
            self.data = np.zeros(4)
            self.data[0] = 1.0
        elif len(args) == 4:
            self.data = np.array(args)
        elif len(args) == 1 and len(args[0]) == 4:
            self.data = np.array(args[0])

    def _getW(self):
        return self.data[0]
    def _setW(self, value):
        self.data[0] = value
    w = property(_getW, _setW)

    def _getX(self):
        return self.data[1]
    def _setX(self, value):
        self.data[1] = value
    x = property(_getX, _setX)

    def _getY(self):
        return self.data[2]
    def _setY(self, value):
        self.data[2] = value
    y = property(_getY, _setY)

    def _getZ(self):
        return self.data[0]
    def _setZ(self, value):
        self.data[0] = value
    z = property(_getZ, _setZ)

class Transform:
    def __init__(self, rot=None, pos=None):
        if not pos is None:
            self.t = np.array(pos)
        else:
            self.t = np.zeros(3)

        if not rot is None:
            self.q = Quaternion()
        elif isinstance(rot ,Quaternion):
            self.q = rot
        else:
            self.q = rotation_to_quaternion(rot)

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
        return Quaternion(quat.getW(), quat.getX(), quat.getY(), quat.getZ())
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

class Contact:
    def __init__(self):
        self.o1 = CollisionGeometry()
        self.o2 = CollisionGeometry()
        self.b1 = 0
        self.b2 = 0
        self.normal = [0.0, 0.0, 0.0]
        self.pos = [0.0, 0.0, 0.0]
        self.penetration_depth = 0.0

class CostSource:
    def __init__(self):
        self.aabb_min = [0.0, 0.0, 0.0]
        self.cost_density = 0.0
        self.total_cost = 0.0

class CollisionResult:
    def __init__(self):
        self.contacts = []
        self.cost_sources = []

class CollisionRequest:
    def __init__(self,
                 num_max_contacts = 1,
                 enable_contact = False,
                 num_max_cost_sources = 1,
                 enable_cost = False,
                 use_approximate_cost = True):
        self.num_max_contacts = num_max_contacts
        self.enable_contact = enable_contact
        self.num_max_cost_sources = num_max_cost_sources
        self.enable_cost = enable_cost
        self.use_approximate_cost = use_approximate_cost

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

cdef python_to_c_contact(defs.Contact contact):
    c = Contact()
    if contact.o1.getNodeType() == defs.GEOM_BOX:
        obj = Box(0, 0, 0)
        memcpy(obj.thisptr, contact.o1, sizeof(defs.Box))
        c.o1 = obj
    elif contact.o1.getNodeType() == defs.GEOM_SPHERE:
        obj = Sphere(0)
        memcpy(obj.thisptr, contact.o1, sizeof(defs.Sphere))
        c.o1 = obj
    elif contact.o1.getNodeType() == defs.GEOM_CAPSULE:
        obj = Capsule(0, 0)
        memcpy(obj.thisptr, contact.o1, sizeof(defs.Capsule))
        c.o1 = obj
    elif contact.o1.getNodeType() == defs.GEOM_CONE:
        obj = Cone(0, 0)
        memcpy(obj.thisptr, contact.o1, sizeof(defs.Cone))
        c.o1 = obj
    elif contact.o1.getNodeType() == defs.GEOM_CYLINDER:
        obj = Cylinder(0, 0)
        memcpy(obj.thisptr, contact.o1, sizeof(defs.Cylinder))
        c.o1 = obj

    if contact.o2.getNodeType() == defs.GEOM_BOX:
        obj = Box(0, 0, 0)
        memcpy(obj.thisptr, contact.o2, sizeof(defs.Box))
        c.o2 = obj
    elif contact.o2.getNodeType() == defs.GEOM_SPHERE:
        obj = Sphere(0)
        memcpy(obj.thisptr, contact.o2, sizeof(defs.Sphere))
        c.o2 = obj
    elif contact.o2.getNodeType() == defs.GEOM_CAPSULE:
        obj = Capsule(0, 0)
        memcpy(obj.thisptr, contact.o2, sizeof(defs.Capsule))
        c.o2 = obj
    elif contact.o2.getNodeType() == defs.GEOM_CONE:
        obj = Cone(0, 0)
        memcpy(obj.thisptr, contact.o2, sizeof(defs.Cone))
        c.o2 = obj
    elif contact.o2.getNodeType() == defs.GEOM_CYLINDER:
        obj = Cylinder(0, 0)
        memcpy(obj.thisptr, contact.o2, sizeof(defs.Cylinder))
        c.o2 = obj
    c.b1 = contact.b1
    c.b2 = contact.b2
    c.normal = vec3f_to_list(contact.normal)
    c.pos = vec3f_to_list(contact.pos)
    c.penetration_depth = contact.penetration_depth
    return c

cdef python_to_c_costsource(defs.CostSource cost_source):
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
        col_res.contacts.append(python_to_c_contact(contacts[idx]))
    for idx in range(costs.size()):
        col_res.cost_sources.append(python_to_c_costsource(costs[idx]))
    return ret, col_res
