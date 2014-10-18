from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.set cimport set

cdef extern from "Python.h":
       ctypedef struct PyObject
       void Py_INCREF(PyObject *obj)
       void Py_DECREF(PyObject *obj)
       object PyObject_CallObject(object obj, object args)
       object PySequence_Concat(object obj1, object obj2)

cdef extern from "boost/shared_ptr.hpp" namespace "boost":
    cppclass shared_ptr[T]:
        shared_ptr() except +
        shared_ptr(T*) except +
        T* get()

cdef extern from "fcl/data_types.h" namespace "fcl":
    ctypedef double FCL_REAL

cdef extern from "fcl/math/vec_3f.h" namespace "fcl":
    cdef cppclass Vec3f:
        Vec3f() except +
        Vec3f(FCL_REAL x, FCL_REAL y, FCL_REAL z) except +
        FCL_REAL& operator[](size_t i)

cdef extern from "fcl/math/matrix_3f.h" namespace "fcl":
    cdef cppclass Matrix3f:
        Matrix3f() except +
        Matrix3f(FCL_REAL xx, FCL_REAL xy, FCL_REAL xz,
                 FCL_REAL yx, FCL_REAL yy, FCL_REAL yz,
                 FCL_REAL zx, FCL_REAL zy, FCL_REAL zz) except +
        FCL_REAL operator()(size_t i, size_t j)

cdef extern from "fcl/math/transform.h" namespace "fcl":
    cdef cppclass Quaternion3f:
        Quaternion3f() except +
        Quaternion3f(FCL_REAL a, FCL_REAL b,
                     FCL_REAL c, FCL_REAL d) except +
        void fromRotation(Matrix3f& R)
        void fromAxisAngle(Vec3f& axis, FCL_REAL angle)
        FCL_REAL& getW()
        FCL_REAL& getX()
        FCL_REAL& getY()
        FCL_REAL& getZ()

    cdef cppclass Transform3f:
        Transform3f() except +
        Transform3f(Matrix3f& R_, Vec3f& T_)
        Transform3f(Quaternion3f& q_, Vec3f& T_)

cdef extern from "fcl/collision_data.h" namespace "fcl":
    cdef cppclass Contact:
        CollisionGeometry *o1
        CollisionGeometry *o2
        int b1
        int b2
        Vec3f normal
        Vec3f pos
        FCL_REAL penetration_depth
        Contact() except +
        Contact(CollisionGeometry* o1_,
                CollisionGeometry* o2_,
                int b1_, int b2_) except +
    cdef cppclass CostSource:
        Vec3f aabb_min
        Vec3f aabb_max
        FCL_REAL cost_density
        FCL_REAL total_cost
    cdef cppclass CollisionResult:
        CollisionResult() except +
        void getContacts(vector[Contact]& contacts_)
        void getCostSources(vector[CostSource]& cost_sources_)
    cdef cppclass CollisionRequest:
        size_t num_max_contacts
        bool enable_contact
        size_t num_max_cost_sources
        bool enable_cost
        bool use_approximate_cost
        CollisionRequest(size_t num_max_contacts_,
                         bool enable_contact_,
                         size_t num_max_cost_sources_,
                         bool enable_cost_,
                         bool use_approximate_cost_)
    cdef cppclass DistanceResult:
        FCL_REAL min_distance
        Vec3f* nearest_points
        CollisionGeometry* o1
        CollisionGeometry* o2
        int b1
        int b2
        DistanceResult(FCL_REAL min_distance_) except +
        DistanceResult() except +
    cdef cppclass DistanceRequest:
        bool enable_nearest_points
        DistanceRequest(bool enable_nearest_points_) except +

cdef extern from "fcl/collision_object.h" namespace "fcl":
    cdef enum OBJECT_TYPE:
        OT_UNKNOWN, OT_BVH, OT_GEOM, OT_OCTREE, OT_COUNT
    cdef enum NODE_TYPE:
        BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24,
        GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE,
        GEOM_HALFSPACE, GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT

    cdef cppclass CollisionGeometry:
        CollisionGeometry() except +
        OBJECT_TYPE getObjectType()
        NODE_TYPE getNodeType()
        void computeLocalAABB()
        Vec3f aabb_center
        FCL_REAL aabb_radius
        FCL_REAL cost_density
        FCL_REAL threshold_occupied
        FCL_REAL threshold_free

    cdef cppclass CollisionObject:
        CollisionObject() except +
        CollisionObject(shared_ptr[CollisionGeometry]& cgeom_) except +
        CollisionObject(shared_ptr[CollisionGeometry]& cgeom_, Transform3f& tf) except +
        OBJECT_TYPE getObjectType()
        NODE_TYPE getNodeType()
        Vec3f& getTranslation()
        Matrix3f& getRotation()
        Quaternion3f& getQuatRotation()
        CollisionGeometry* getCollisionGeometry()
        void setTranslation(Vec3f& T)
        void setQuatRotation(Quaternion3f& q)
        void setTransform(Quaternion3f& q, Vec3f& T)
        bool isOccupied()
        bool isFree()
        bool isUncertain()

    ctypedef CollisionGeometry const_CollisionGeometry "const fcl::CollisionGeometry"
    ctypedef CollisionObject const_CollisionObject "const fcl::CollisionObject"

cdef extern from "fcl/shape/geometric_shapes.h" namespace "fcl":
    cdef cppclass ShapeBase(CollisionGeometry):
        ShapeBase() except +

    cdef cppclass TriangleP(ShapeBase):
        TriangleP(Vec3f& a_, Vec3f& b_, Vec3f& c_) except +
        Vec3f a, b, c

    cdef cppclass Box(ShapeBase):
        Box(FCL_REAL x, FCL_REAL y, FCL_REAL z) except +
        Vec3f side

    cdef cppclass Sphere(ShapeBase):
        Sphere(FCL_REAL radius_) except +
        FCL_REAL radius

    cdef cppclass Capsule(ShapeBase):
        Capsule(FCL_REAL radius_, FCL_REAL lz_) except +
        FCL_REAL radius
        FCL_REAL lz

    cdef cppclass Cone(ShapeBase):
        Cone(FCL_REAL radius_, FCL_REAL lz_) except +
        FCL_REAL radius
        FCL_REAL lz

    cdef cppclass Cylinder(ShapeBase):
        Cylinder(FCL_REAL radius_, FCL_REAL lz_) except +
        FCL_REAL radius
        FCL_REAL lz

    cdef cppclass Convex(ShapeBase):
        Convex(Vec3f* plane_nomals_,
               FCL_REAL* plane_dis_,
               int num_planes,
               Vec3f* points_,
               int num_points_,
               int* polygons_) except +

    cdef cppclass Halfspace(ShapeBase):
        Halfspace(Vec3f& n_, FCL_REAL d_) except +
        Vec3f n
        FCL_REAL d

    cdef cppclass Plane(ShapeBase):
        Plane(Vec3f& n_, FCL_REAL d_) except +
        Vec3f n
        FCL_REAL d

cdef extern from "fcl/broadphase/broadphase.h" namespace "fcl":
    ctypedef bool (*CollisionCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata)
    ctypedef bool (*DistanceCallBack)(CollisionObject* o1, CollisionObject* o2, void* cdata, FCL_REAL& dist)

cdef extern from "fcl/broadphase/broadphase_dynamic_AABB_tree.h" namespace "fcl":
    cdef cppclass DynamicAABBTreeCollisionManager:
        DynamicAABBTreeCollisionManager() except +
        void registerObjects(vector[CollisionObject*]& other_objs)
        void registerObject(CollisionObject* obj)
        void unregisterObject(CollisionObject* obj)
        void collide(CollisionObject* obj, void* cdata, CollisionCallBack callback)
        void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback)
        void collide(void* cdata, CollisionCallBack callback)
        void distance(void* cdata, DistanceCallBack callback)
        void setup()
        void update()
        void update(CollisionObject* updated_obj)
        void update(vector[CollisionObject*] updated_objs)
        void clear()
        bool empty()
        size_t size()
        int max_tree_nonbalanced_level
        int tree_incremental_balance_pass
        int& tree_topdown_balance_threshold
        int& tree_topdown_level
        int tree_init_level
        bool octree_as_geometry_collide
        bool octree_as_geometry_distance

cdef extern from "fcl/collision.h" namespace "fcl":
    size_t collide(CollisionObject* o1, CollisionObject* o2,
                   CollisionRequest& request,
                   CollisionResult& result)
    size_t collide(CollisionGeometry* o1, Transform3f& tf1,
                   CollisionGeometry* o2, Transform3f& tf2,
                   CollisionRequest& request,
                   CollisionResult& result)

cdef extern from "fcl/distance.h" namespace "fcl":
    FCL_REAL distance(CollisionObject* o1, CollisionObject* o2,
                      DistanceRequest& request, DistanceResult& result)
    FCL_REAL distance(CollisionGeometry* o1, Transform3f& tf1,
                      CollisionGeometry* o2, Transform3f& tf2,
                      DistanceRequest& request, DistanceResult& result)
