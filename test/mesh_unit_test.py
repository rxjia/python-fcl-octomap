from unittest import TestCase
from fcl import mesh


class Test_BVHModel(TestCase):
    def setUp(self):
        self.mesh = mesh.BVHModel()

    def test_num_tris(self):
        print self.mesh

        print "node?", self.mesh.node()

        # print self.mesh.getNumBVs()
        # print self.mesh.xxx
        # self.mesh.num_tris = 12
        # self.assertEqual(self.mesh.num_tris, 12)


