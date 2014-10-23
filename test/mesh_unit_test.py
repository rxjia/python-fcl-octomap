from unittest import TestCase

from fcl import fcl


class Test_BVHModel(TestCase):
    def setUp(self):
        print "create new bvh model"
        self.mesh = fcl.BVHModel()

    def create_occ_box(self):
        from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
        self.box = BRepPrimAPI_MakeBox(10, 20, 30).Shape()

    def test_get_mesh_data_from_occ_brep(self, occ_brep=None):
        pass

        # # todo: create a numpy property of the vertex, normal and tri's index arrays...
        # from OCC.Visualization import Tesselator, atNormal
        # tess = Tesselator(occ_brep, atNormal, 1.0, 1, 0.01, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.)
        #
        # self.n_tris = tess.ObjGetTriangleCount()
        # self.n_normals = tess.ObjGetNormalCount()
        # self.vertices = tess.VerticesList()

        # if not hasattr(self, "box"):
        #     self.create_occ_box()
        #
        # from OCC.BRep import BRep_Tool_Triangulation
        # from OCC.TopAbs import TopAbs_FACE
        # from OCC.TopExp import TopExp_Explorer
        # from OCC.TopLoc import TopLoc_Location
        # from OCC.BRepMesh import brepmesh, BRepMesh_ReMesh, BRepMesh_IncrementalMesh
        # from OCC.TopoDS import topods_Face
        #
        # # brepmesh.Mesh(self.box, 0.8)
        # # BRepMesh_ReMesh(self.box, 0.8)
        # inc_mesh = BRepMesh_IncrementalMesh(self.box, 0.8)
        # assert inc_mesh.IsDone()
        #
        #
        # ex = TopExp_Explorer(self.box, TopAbs_FACE)
        # while ex.More():
        #     F = topods_Face(ex.Current())
        #     L = TopLoc_Location()
        #     facing = (BRep_Tool_Triangulation(F, L)).GetObject()
        #     tri = facing.Triangles()
        #     for i in range(1, facing.NbTriangles() + 1):
        #         trian = tri.Value(i)
        #         index1, index2, index3 = trian.Get()
        #     ex.Next()

    #
    def test_getNodeType(self):
        # BV_OBBRSS -> 5
        node_type = self.mesh.getNodeType()
        self.assertEqual(node_type, 5, "expected node type 5, got {0}".format(node_type))
    #
    # def test_num_tris(self):
    #     self.assertEqual(self.mesh.num_tries_(), 0)
    #     print self.mesh.num_tris
    #     # print "node?", self.mesh.thisptr
    #
    #     # print self.mesh.getNumBVs()
    #     # print self.mesh.xxx
    #     # self.mesh.num_tris = 12
    #     # self.assertEqual(self.mesh.num_tris, 12)
    #
    # def test_buildState(self):
    #     print self.mesh.buildState()
    #
    #
    # def test_addSubModel(self):
    #     self.mesh.beginModel(1,1)
    #     tris = self.mesh.num_tries_()
    #     self.mesh.addSubModel(self.vertices, self.triangles)
    #     self.assertEqual(tris, 1, "expected 1 triangle, got {0}".format(tris))
    #     print self.mesh.endModel()
    #
    #
    # def test_beginModel(self):
    #     self.assertTrue(self.mesh.beginModel(1,1), 0)

    def test_addTriangle(self):
        print 'add triangle'
        self.mesh.beginModel(1,1)
        self.assertEqual(self.mesh.addTriangle([0,0,0],[1,1,1],[2,2,2]), True)
        self.mesh.endModel()
        self.assertEqual(self.mesh.num_tries_(), 1)
        # note no self.mesh.beginModel call
        # ValueError: BVH construction does not follow correct sequence
        with self.assertRaises(ValueError):
            self.mesh.addTriangle([0,0,0],[1,1,1],[2,2,2])
        self.mesh.endModel()

    def test_addVertex(self):
        print 'add vertex'
        model = self.mesh.beginModel(1, 1)
        self.assertEqual(model, 0, msg="add vertex, begin model")
        v = self.mesh.addVertex(100,0,10)
        self.assertTrue(v, "expected True")
        # self.assertRaises(ValueError, self.mesh.endModel())
        vv=self.mesh.endModel()
        print "vv:",vv
        # self.assertEqual(model, 0, "add vertex, end model, expected 0 got {0}".format(model))

    def test_del(self):
        del self.mesh

    def test_collide_Triangle_Box(self):
        from fcl import fcl

        self.box = fcl.Box(10,10,10)
        self.bvh = fcl.BVHModel()

        self.bvh.beginModel(1,1)
        self.bvh.addTriangle([-1,1,1], [11,11,1], [0,0,0])
        self.bvh.endModel()

        coll_bvh = fcl.CollisionObject(self.bvh)
        # coll_box = fcl.CollisionObject(self.box)
        # ret, result = fcl.collide(coll_box,
        #                           coll_bvh,
        #                           fcl.cd.CollisionRequest()
        # )
        # print coll_bvh

        # self.assertEqual(result.contacts, [])
        # self.assertEqual(result.cost_sources, [])
        #
        # dis, result = fcl.distance(fcl.CollisionObject(self.box),
        #                            fcl.CollisionObject(self.bvh),
        #                            fcl.cd.DistanceRequest(True)
        # )




