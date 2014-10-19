from libcpp cimport bool
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport fcl_defs as defs
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

    def node(self):
        return self.thisptr.getNodeType()

    #     # return self.thisptr.getNodeType()
    #
    # property xxx:
    #     def __get__(self):
    #         # return self.thisptr.num_tris
    #         # return self.thisptr.num_tris
    #         return (<defs.BVHModel*> self.thisptr).num_tris
    #     def __set__(self, value):
    #         self.num_tris = <int?> value
    #         # (<defs.BVHModel*> self.thisptr).num_tris = <int?> value

    # def __str__(self):
        # return <str?>self.thisptr

# cdef class DynamicAABBTreeCollisionManager:
#     cdef defs.DynamicAABBTreeCollisionManager *thisptr
#     cdef vector[defs.PyObject*]*objs
#     def __cinit__(self):
#         self.thisptr = new defs.DynamicAABBTreeCollisionManager()
#         self.objs = new vector[defs.PyObject *]()


# cdef class OBBRSS:
#     cdef defs.OBBRSS *thisptr
#     def __cinit__(self, n, d):
#         self.thisptr = new defs.OBBRSS()
