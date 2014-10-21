from unittest import TestCase

from fcl import mesh


class Test_BVHModel(TestCase):
    def setUp(self):
        self.mesh = mesh.BVHModel()

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


    def test_getNodeType(self):
        self.assertEqual(self.mesh.getNodeType(), 5)

    def test_num_tris(self):
        self.assertEqual(self.mesh.num_tries_(), 0)
        print self.mesh.num_tris
        # print "node?", self.mesh.thisptr

        # print self.mesh.getNumBVs()
        # print self.mesh.xxx
        # self.mesh.num_tris = 12
        # self.assertEqual(self.mesh.num_tris, 12)

    def test_buildState(self):
        print self.mesh.buildState()


    def test_addSubModel(self):
        print self.mesh.beginModel()
        self.mesh.addSubModel(self.vertices, self.triangles)
        print self.mesh.endModel()




        # def test_addVertex(self):
        # print self.mesh.addVertex(100,0,10)

