import sys

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

class DistanceResult:
    def __init__(self, min_distance_ = sys.float_info.max):
        self.min_distance = min_distance_
        self.nearest_points = [None, None]
        self.o1 = None
        self.o2 = None
        self.b1 = -1
        self.b2 = -1

class DistanceRequest:
    def __init__(self,
                 enable_nearest_points_ = False):
        self.enable_nearest_points = enable_nearest_points_
