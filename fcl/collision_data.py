import sys

class OBJECT_TYPE:
    OT_UNKNOWN, OT_BVH, OT_GEOM, OT_OCTREE, OT_COUNT = range(5)


class NODE_TYPE:
    BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24, \
    GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE, \
    GEOM_HALFSPACE, GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT = range(20)


class NODE_TYPE:
    BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24, \
    GEOM_BOX, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE, \
    GEOM_HALFSPACE, GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT = range(20)


class CCDMotionType:
    CCDM_TRANS, CCDM_LINEAR, CCDM_SCREW, CCDM_SPLINE = range(4)


class CCDSolverType:
    CCDC_NAIVE, CCDC_CONSERVATIVE_ADVANCEMENT, CCDC_RAY_SHOOTING, CCDC_POLYNOMIAL_SOLVER = range(4)


class GJKSolverType:
    GST_LIBCCD, GST_INDEP = range(2)


class CostSource:
    def __init__(self):
        self.aabb_min = [0.0, 0.0, 0.0]
        self.cost_density = 0.0
        self.total_cost = 0.0


class CollisionResult:
    def __init__(self):
        self.contacts = []
        self.cost_sources = []


class ContinuousCollisionResult:
    def __init__(self, is_collide=False, time_of_contact=1.0):
        self.is_collide = is_collide
        self.time_of_contact = time_of_contact

class CollisionRequest:
    def __init__(self,
                 num_max_contacts=1,
                 enable_contact=False,
                 num_max_cost_sources=1,
                 enable_cost=False,
                 use_approximate_cost=True):
        self.num_max_contacts = num_max_contacts
        self.enable_contact = enable_contact
        self.num_max_cost_sources = num_max_cost_sources
        self.enable_cost = enable_cost
        self.use_approximate_cost = use_approximate_cost


class ContinuousCollisionRequest:
    def __init__(self,
                 num_max_iterations=10,
                 toc_err=0.0001,
                 ccd_motion_type=CCDMotionType.CCDM_TRANS,
                 gjk_solver_type=GJKSolverType.GST_LIBCCD,
                 ccd_solver_type=CCDSolverType.CCDC_CONSERVATIVE_ADVANCEMENT):
        self.num_max_iterations = num_max_iterations
        self.toc_err = toc_err
        self.ccd_motion_type = ccd_motion_type
        self.gjk_solver_type = gjk_solver_type
        self.ccd_solver_type = ccd_solver_type

class DistanceResult:
    def __init__(self, min_distance_=sys.float_info.max):
        self.min_distance = min_distance_
        self.nearest_points = [None, None]
        self.o1 = None
        self.o2 = None
        self.b1 = -1
        self.b2 = -1


class DistanceRequest:
    def __init__(self,
                 enable_nearest_points_=False):
        self.enable_nearest_points = enable_nearest_points_
