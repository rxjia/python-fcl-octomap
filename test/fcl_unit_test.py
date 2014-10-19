import sys
from unittest import TestCase
from fcl import fcl, transform


class Test_FCL(TestCase):
    def setUp(self):
        self.objs = [fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0)),
                     fcl.CollisionObject(fcl.Sphere(4.0)),
                     fcl.CollisionObject(fcl.Cone(5.0, 6.0))]

        self.manager = fcl.DynamicAABBTreeCollisionManager()
        self.res = fcl.CollisionResult()
        # self.manager.collide(self.res, self.collide_callback_func)

        self._side = (1.0, 2.0, 3.0)
        self._radius = 4.0
        self._nearest_points = [(-0.5, 0.0, 0.0), (4.0, 0.0, 0.0)]
        self._ref_dist = 15.5
        self.pt1 = [10.0, 0.0, 0.0]
        self.pt2 = [-10.0, 0.0, 0.0]

        self.box = fcl.Box(*self._side)
        self.sphere = fcl.Sphere(self._radius)
        self.cyl = fcl.Cylinder(7.0, 8.0)

        self.trans1 = transform.Transform(transform.Quaternion(), self.pt1)
        self.trans2 = transform.Transform(transform.Quaternion(), self.pt2)

    def test_dynamic_aabb_tree_collision_manager(self):
        _size = 0
        self.assertTrue(self.manager.size() == 0)
        self.manager.registerObjects(self.objs)
        _size += len(self.objs)
        self.assertTrue(self.manager.size() == _size)
        self.manager.registerObject(fcl.CollisionObject(self.cyl))
        _size += 1
        self.assertTrue(self.manager.size() == _size)

    def collide_callback_func(obj1, obj2, res):
        ret, res = fcl.collide(obj1, obj2, fcl.CollisionRequest())
        return ret

    def test_distance_box_sphere_translated(self):
        ret, result = fcl.collide(fcl.CollisionObject(self.box, self.trans1),
                                  fcl.CollisionObject(self.sphere, self.trans2),
                                  fcl.CollisionRequest()
        )

        self.assertEqual(result.contacts, [])
        self.assertEqual(result.cost_sources, [])

        dis, result = fcl.distance(fcl.CollisionObject(self.box, self.trans1),
                                   fcl.CollisionObject(self.sphere, self.trans2),
                                   fcl.DistanceRequest(True)
        )

        self.assertEqual(dis, self._ref_dist)
        self.assertEqual(result.min_distance, self._ref_dist)
        self.assertEqual(result.nearest_points, self._nearest_points)
        self.assertEqual(result.o2.radius, self._radius)
        self.assertEqual(result.o1.side, self._side)

    def test_collision_box_sphere(self):
        ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(*self._side),
                                                      transform.Transform(transform.Quaternion())),
                                  fcl.CollisionObject(fcl.Sphere(self._radius),
                                                      transform.Transform(transform.Quaternion(), [0.0, 0.0, 0.0])),
                                  fcl.CollisionRequest())

        self.assertTrue(len(result.contacts) == 1)
        self.assertTrue(len(result.cost_sources) == 0)

        con = result.contacts[0]
        self.assertAlmostEqual(con.penetration_depth, 0.0)
        self.assertTrue(con.penetration_depth < sys.float_info.min)
        self.assertTrue(isinstance(con.o1, fcl.Box))
        self.assertTrue(isinstance(con.o2, fcl.Sphere))

    def test_collision_and_distance_box_sphere(self):
        dis, result = fcl.distance(fcl.CollisionObject(self.box, self.trans1),
                                   fcl.CollisionObject(self.sphere, self.trans2),
                                   fcl.DistanceRequest(True)
        )

        self.assertEqual(dis, self._ref_dist)
        self.assertEqual(result.min_distance, self._ref_dist)
        self.assertEqual(result.nearest_points, self._nearest_points)
        self.assertEqual(result.b1, -1)
        self.assertEqual(result.b2, -1)

        dis, result = fcl.distance(fcl.CollisionObject(self.box, transform.Transform(transform.Quaternion())),
                                   fcl.CollisionObject(self.sphere,
                                                       transform.Transform(transform.Quaternion(), [0.0, 0.0, 0.0])),
                                   fcl.DistanceRequest(True)
        )

        self.assertEqual(dis, -1.0)
        self.assertEqual(result.min_distance, -1.0)

        for i in result.nearest_points:
            for j in i:
                self.assertAlmostEqual(0.0, j)

    def test_triangle(self):
        """

            TriangleP is not well supported...

            Perhaps not much of an issue, since BVHModel will deal with meshes / triangle soup
            So, what'evs?

        """
        import numpy as np

        li = [[0, 100, 100], [0, 0, 0], [100, 0, 100], ]
        arr = np.array(li, "f")
        _p = fcl.TriangleP(arr[0], arr[1], arr[2])
        p = fcl.CollisionObject(_p)

        collision_object = fcl.CollisionObject(self.box, transform.Transform(transform.Quaternion()))

        # TODO: segfault!
        # ....Warning: distance function between node type 9 and node type 17 is not supported
        # dis, result = fcl.distance(collision_object, p, fcl.DistanceRequest(True))

        ret, result = fcl.collide(collision_object, p, fcl.CollisionRequest())
        # Warning: collision function between node type 9 and node type 17 is not supported
        self.assertTrue(ret==0)



