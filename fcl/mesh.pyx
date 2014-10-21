from libcpp cimport bool
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport fcl_defs as defs
cimport fcl
cimport numpy as np
ctypedef np.float64_t DOUBLE_t



#-------------------------------------------------------------------------------
# templates are handled pretty ugly...
# https://groups.google.com/forum/#!searchin/cython-users/ctypedef$20template/cython-users/40JVog15WS4/LwJk-9jj4OIJ
#-------------------------------------------------------------------------------

cdef class BVHModel:
    cdef defs.BVHModel *thisptr
    def __cinit__(self):
        self.thisptr = new defs.BVHModel()

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def num_tries_(self):
        return self.thisptr.num_tris

    def buildState(self):
        return self.thisptr.build_state

    def addVertex(self, x,y,z):
        self.thisptr.addVertex( defs.Vec3f(<double?>x, <double?>y, <double?>z, ) )


    # def getNodeType(self):
    #     if self.thisptr:
    #         return self.thisptr.getNodeType()
    #     else:
    #         return None
    #
    # def computeLocalAABB(self):
    #     if self.thisptr:
    #         self.thisptr.computeLocalAABB()
    #     else:
    #         return None

    # def beginModel( self, num_tris_, num_vertices_):
    #     self.thisptr.beginModel(<int?>num_tris_, <int?>num_vertices_)

    # property aabb_center:
    #     def __get__(self):
    #         if self.thisptr:
    #             return fcl.vec3f_to_tuple(self.thisptr.aabb_center)
    #         else:
    #             return None
    #     def __set__(self, value):
    #         if self.thisptr:
    #             self.thisptr.aabb_center[0] = value[0]
    #             self.thisptr.aabb_center[1] = value[1]
    #             self.thisptr.aabb_center[2] = value[2]
    #         else:
    #             raise ReferenceError


#-------------------------------------------------------------------------------
# TODO
#-------------------------------------------------------------------------------


# cdef class ContinuousCollisionRequest:
#     cdef defs.ContinuousCollisionRequest *thisptr
#
#     def __cinit__(self):
#         self.thisptr = new defs.ContinuousCollisionRequest()
#

# continuousCollide(o1, tf_goal_o1, o2, tf_goal_o2, request, result);