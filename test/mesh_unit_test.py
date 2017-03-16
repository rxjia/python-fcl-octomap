from unittest import TestCase
import collections


class Test_BVHModel(TestCase):
    def setUp(self):
        from fcl import fcl

        print("create new bvh model")
        self.mesh = fcl.BVHModel()

    def create_occ_box(self):
        try:
            from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
            self.box = BRepPrimAPI_MakeBox(10, 20, 30).Shape()
        except ImportError:
            print("pythonocc not installed")

    def test_get_mesh_data_from_occ_brep(self, occ_brep=None):
        try:
            from OCC.BRep import BRep_Tool_Triangulation
            from OCC.TopAbs import TopAbs_FACE
            from OCC.TopExp import TopExp_Explorer
            from OCC.TopLoc import TopLoc_Location
            from OCC.BRepMesh import BRepMesh_IncrementalMesh
            from OCC.TopoDS import TopoDS_face

        except ImportError:
            print("looks like pythonocc is not installed")

        else:
            self.create_occ_box()
            inc_mesh = BRepMesh_IncrementalMesh(self.box, 0.8)
            assert inc_mesh.IsDone()

            ex = TopExp_Explorer(self.box, TopAbs_FACE)
            triangles = collections.deque()
            while ex.More():
                F = TopoDS_face(ex.Current())
                L = TopLoc_Location()
                facing = (BRep_Tool_Triangulation(F, L)).GetObject()
                tri = facing.Triangles()
                nodes = facing.Nodes()
                for i in range(1, facing.NbTriangles() + 1):
                    trian = tri.Value(i)
                    index1, index2, index3 = trian.Get()
                    triangles.append(( nodes.Value(index1), nodes.Value(index1), nodes.Value(index1) ))
                ex.Next()


    def test_getNodeType(self):
        # BV_OBBRSS -> 5
        node_type = self.mesh.getNodeType()
        self.assertEqual(node_type, 5, "expected node type 5, got {0}".format(node_type))


    def test_addTriangle(self):
        print('add triangle')
        self.mesh.beginModel(1, 1)
        self.assertEqual(self.mesh.addTriangle([0, 0, 0], [1, 1, 1], [2, 2, 2]), True)
        self.mesh.endModel()
        self.assertEqual(self.mesh.num_tries_(), 1)
        # note no self.mesh.beginModel call
        # ValueError: BVH construction does not follow correct sequence
        self.assertRaises(ValueError, self.mesh.addTriangle, [0, 0, 0], [1, 1, 1], [2, 2, 2])
        self.mesh.endModel()

    def test_addVertex(self):
        print('add vertex')
        model = self.mesh.beginModel(1, 1)
        self.assertEqual(model, 0, msg="add vertex, begin model")
        v = self.mesh.addVertex(100, 0, 10)
        self.assertTrue(v, "expected True")
        # self.assertRaises(ValueError, self.mesh.endModel())
        vv = self.mesh.endModel()
        print("vv:", vv)
        # self.assertEqual(model, 0, "add vertex, end model, expected 0 got {0}".format(model))

    def test_del(self):
        del self.mesh

    def test_collide_Triangle_Box(self):
        from fcl import fcl
        from fcl import collision_data as cd

        self.box = fcl.Box(10, 10, 10)

        self.bvh = fcl.BVHModel()
        self.bvh.beginModel(1, 1)
        self.bvh.addTriangle([-1, 1, 1], [11, 11, 1], [0, 0, 0])
        self.bvh.endModel()

        coll_bvh = fcl.CollisionObject(self.bvh)
        coll_box = fcl.CollisionObject(self.box)

        ret, result = fcl.distance(coll_box, coll_bvh, cd.DistanceRequest(True))
        # TODO this has probably failed? the -1
        self.assertEqual(ret, -1)

        ret, result = fcl.collide(coll_box, coll_bvh, cd.CollisionRequest())
        self.assertEqual(ret, 1)
        self.assertAlmostEqual(result.contacts[0].penetration_depth, -0.5634, places=3)

    def test_continuous_collide(self):
        from fcl import fcl
        from fcl import collision_data as cd
        from fcl import transform as tf

        box = fcl.Box(1, 1, 1)
        sph = fcl.Cylinder(1, 1)


        no_trans = tf.Transform()
        trans_X_neg = tf.Transform(tf.Quaternion(), [-10, 0, 0])
        trans_X_pos = tf.Transform(tf.Quaternion(), [10, 0, 0])

        coll_box_A = fcl.CollisionObject(box) #, trans_X_neg)
        coll_box_A.setTranslation(trans_X_neg.t)

        coll_box_B = fcl.CollisionObject(box) #, trans_X_pos)
        coll_box_B.setTranslation(trans_X_pos.t)

        coll_sph = fcl.CollisionObject(sph, no_trans)

        request = cd.ContinuousCollisionRequest()

        ret, result = fcl.continuousCollide(coll_box_A, trans_X_neg, coll_box_B, trans_X_pos, request)
        self.assertTrue(result.is_collide is False)

        ret, result = fcl.continuousCollide(fcl.CollisionObject(box, no_trans), no_trans,
                                            fcl.CollisionObject(sph, no_trans), no_trans, request)
        self.assertTrue(result.is_collide is True)

