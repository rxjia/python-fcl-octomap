import unittest
import fcl
import sys

import numpy as np

class TestFCL(unittest.TestCase):

    def setUp(self):
        verts = np.array([[0.0, 0.0, 0.0],
                          [1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0]])
        tris  = np.array([[0,2,1],
                          [0,3,2],
                          [0,1,3],
                          [1,2,3]])
        mesh = fcl.BVHModel()
        mesh.beginModel(len(verts), len(tris))
        mesh.addSubModel(verts, tris)
        mesh.endModel()

        self.geometry = {
            'box'      : fcl.Box(1.0, 1.0, 1.0),
            'sphere'   : fcl.Sphere(1.0),
            'cone'     : fcl.Cone(1.0, 1.0),
            'cylinder' : fcl.Cylinder(1.0, 1.0),
            'mesh'     : mesh
        }

        self.crequest = fcl.CollisionRequest(
            num_max_contacts = 100,
            enable_contact = True
        )
        self.drequest = fcl.DistanceRequest(
            enable_nearest_points = True
        )

        self.x_axis_rot = np.array([[1.0, 0.0, 0.0],
                                    [0.0, 0.0, -1.0],
                                    [0.0, 1.0, 0.0]])


    def test_pairwise_collisions(self):
        result = fcl.CollisionResult()

        box = fcl.CollisionObject(self.geometry['box'])
        cone = fcl.CollisionObject(self.geometry['cone'])
        mesh = fcl.CollisionObject(self.geometry['mesh'])

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(box, mesh, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(cone, mesh, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        cone.setTranslation(np.array([0.0, 0.0, -0.6]))

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(cone, mesh, self.crequest, result)
        self.assertTrue(ret == 0)
        self.assertFalse(result.is_collision)

        cone.setTranslation(np.array([0.0, -0.9, 0.0]))
        cone.setRotation(self.x_axis_rot)

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        cone.setTranslation(np.array([0.0, -1.1, 0.0]))

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret == 0)
        self.assertFalse(result.is_collision)

    def test_pairwise_distances(self):
        result = fcl.DistanceResult()

        box = fcl.CollisionObject(self.geometry['box'])
        cone = fcl.CollisionObject(self.geometry['cone'])
        mesh = fcl.CollisionObject(self.geometry['mesh'])

        result = fcl.DistanceResult()
        ret = fcl.distance(box, cone, self.drequest, result)
        self.assertTrue(ret < 0)

        result = fcl.DistanceResult()
        ret = fcl.distance(box, mesh, self.drequest, result)
        self.assertTrue(ret < 0)

        result = fcl.DistanceResult()
        ret = fcl.distance(cone, mesh, self.drequest, result)
        self.assertTrue(ret < 0)

        cone.setTranslation(np.array([0.0, 0.0, -0.6]))

        result = fcl.DistanceResult()
        ret = fcl.distance(box, cone, self.drequest, result)
        self.assertTrue(ret < 0)

        result = fcl.DistanceResult()
        ret = fcl.distance(cone, mesh, self.drequest, result)
        self.assertAlmostEqual(ret, 0.1)

        cone.setTranslation(np.array([0.0, -0.9, 0.0]))
        cone.setRotation(self.x_axis_rot)

        result = fcl.DistanceResult()
        ret = fcl.distance(box, cone, self.drequest, result)
        self.assertTrue(ret < 0)

        cone.setTranslation(np.array([0.0, -1.1, 0.0]))

        result = fcl.DistanceResult()
        ret = fcl.distance(box, cone, self.drequest, result)
        self.assertAlmostEqual(ret, 0.1)

    def test_pairwise_continuous_collisions(self):
        request = fcl.ContinuousCollisionRequest()
        result = fcl.ContinuousCollisionResult()

        box = fcl.CollisionObject(self.geometry['box'])
        cone = fcl.CollisionObject(self.geometry['cone'],
                                   fcl.Transform(np.array([0.0, 0.0, -2.0])))

        ret = fcl.continuousCollide(box, fcl.Transform(),
                                    cone, fcl.Transform(),
                                    request, result)

        '''
        ## WHY DOES THIS FAIL ##
        self.assertTrue(result.is_collide)
        self.assertAlmostEqual(0.625, ret)
        '''

    def test_managed_collisions(self):
        manager1 = fcl.DynamicAABBTreeCollisionManager()
        manager2 = fcl.DynamicAABBTreeCollisionManager()
        manager3 = fcl.DynamicAABBTreeCollisionManager()

        objs1 = [fcl.CollisionObject(self.geometry['box']),
                 fcl.CollisionObject(self.geometry['cylinder'])]
        objs2 = [fcl.CollisionObject(self.geometry['cone'], fcl.Transform(np.array([0.0, 0.0, 5.0]))),
                 fcl.CollisionObject(self.geometry['cylinder'], fcl.Transform(np.array([0.0, 0.0, -5.0])))]
        objs3 = [fcl.CollisionObject(self.geometry['mesh']),
                 fcl.CollisionObject(self.geometry['box'], fcl.Transform(np.array([0.0, 0.0, -5.0])))]

        manager1.registerObjects(objs1)
        manager2.registerObjects(objs2)
        manager3.registerObjects(objs3)

        manager1.setup()
        manager2.setup()
        manager3.setup()

        self.assertTrue(len(manager1.getObjects()) == 2)
        self.assertTrue(len(manager2.getObjects()) == 2)
        self.assertTrue(len(manager3.getObjects()) == 2)


        # One-to-many
        o1 = fcl.CollisionObject(self.geometry['box'])
        o2 = fcl.CollisionObject(self.geometry['cylinder'], fcl.Transform(np.array([0.0, 0.0, -4.6])))

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(o1, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(o2, cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(o1, cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(o2, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        # Many-to-many, internal
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        # Many-to-many, grouped
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(manager2, cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(manager3, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(manager3, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

    def test_updates(self):
        manager = fcl.DynamicAABBTreeCollisionManager()

        objs = [fcl.CollisionObject(self.geometry['sphere']),
                fcl.CollisionObject(self.geometry['sphere'], fcl.Transform(np.array([0.0, 0.0, -5.0])))]

        manager.registerObjects(objs)
        manager.setup()

        self.assertTrue(len(manager.getObjects()) == 2)

        # Many-to-many, internal
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        objs[1].setTranslation(np.array([0.0, 0.0, -0.3]))
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.update(objs[1])
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

    def test_many_objects(self):
        manager = fcl.DynamicAABBTreeCollisionManager()

        objs = [fcl.CollisionObject(self.geometry['sphere']),
                fcl.CollisionObject(self.geometry['sphere'], fcl.Transform(np.array([0.0, 0.0, -5.0]))),
                fcl.CollisionObject(self.geometry['sphere'], fcl.Transform(np.array([0.0, 0.0, 5.0])))]

        manager.registerObjects(objs)
        manager.setup()

        self.assertTrue(len(manager.getObjects()) == 3)

        # Many-to-many, internal
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        objs[1].setTranslation(np.array([0.0, 0.0, -0.3]))
        manager.update(objs[1])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        objs[1].setTranslation(np.array([0.0, 0.0, -5.0]))
        manager.update(objs[1])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        objs[2].setTranslation(np.array([0.0, 0.0, 0.3]))
        manager.update(objs[2])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        objs[2].setTranslation(np.array([0.0, 0.0, 5.0]))
        manager.update(objs[2])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

    def test_managed_distances(self):
        manager1 = fcl.DynamicAABBTreeCollisionManager()
        manager2 = fcl.DynamicAABBTreeCollisionManager()
        manager3 = fcl.DynamicAABBTreeCollisionManager()

        objs1 = [fcl.CollisionObject(self.geometry['box']),
                 fcl.CollisionObject(self.geometry['cylinder'])]
        objs2 = [fcl.CollisionObject(self.geometry['cone'], fcl.Transform(np.array([0.0, 0.0, 5.0]))),
                 fcl.CollisionObject(self.geometry['cylinder'], fcl.Transform(np.array([0.0, 0.0, -5.0])))]
        objs3 = [fcl.CollisionObject(self.geometry['mesh']),
                 fcl.CollisionObject(self.geometry['box'], fcl.Transform(np.array([0.0, 0.0, -5.0])))]

        manager1.registerObjects(objs1)
        manager2.registerObjects(objs2)
        manager3.registerObjects(objs3)

        manager1.setup()
        manager2.setup()
        manager3.setup()

        self.assertTrue(len(manager1.getObjects()) == 2)
        self.assertTrue(len(manager2.getObjects()) == 2)
        self.assertTrue(len(manager3.getObjects()) == 2)

        # One-to-many
        o1 = fcl.CollisionObject(self.geometry['box'])
        o2 = fcl.CollisionObject(self.geometry['cylinder'],
                                 fcl.Transform(np.array([0.0, 0.0, -4.6])))

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(o1, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(o2, cdata, fcl.defaultDistanceCallback)
        assert abs(cdata.result.min_distance - 3.6) < 1e-4
        
        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(o1, cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 4.0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(o2, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        # Many-to-many, internal
        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 9.0)

        # Many-to-many, grouped
        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(manager2, cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 4.0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(manager3, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(manager3, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

if __name__ == '__main__':
    unittest.main()
