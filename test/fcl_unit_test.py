import sys
from unittest import TestCase
from fcl import fcl, transform
from fcl import collision_data as cd

class Test_FCL(TestCase):
    def setUp(self):
        self.objs = [fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0)),
                     fcl.CollisionObject(fcl.Sphere(4.0)),
                     fcl.CollisionObject(fcl.Cone(5.0, 6.0))]

        self.manager = fcl.DynamicAABBTreeCollisionManager()
        self.res = cd.CollisionResult()
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
        ret, res = fcl.collide(obj1, obj2, cd.CollisionRequest())
        return ret

    def test_distance_box_sphere_translated(self):
        ret, result = fcl.collide(fcl.CollisionObject(self.box, self.trans1),
                                  fcl.CollisionObject(self.sphere, self.trans2),
                                  cd.CollisionRequest()
        )

        self.assertEqual(result.contacts, [])
        self.assertEqual(result.cost_sources, [])

        dis, result = fcl.distance(fcl.CollisionObject(self.box, self.trans1),
                                   fcl.CollisionObject(self.sphere, self.trans2),
                                   cd.DistanceRequest(True)
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
                                  cd.CollisionRequest())

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
                                   cd.DistanceRequest(True)
        )

        self.assertEqual(dis, self._ref_dist)
        self.assertEqual(result.min_distance, self._ref_dist)
        self.assertEqual(result.nearest_points, self._nearest_points)
        self.assertEqual(result.b1, -1)
        self.assertEqual(result.b2, -1)

        dis, result = fcl.distance(fcl.CollisionObject(self.box, transform.Transform(transform.Quaternion())),
                                   fcl.CollisionObject(self.sphere,
                                                       transform.Transform(transform.Quaternion(), [0.0, 0.0, 0.0])),
                                   cd.DistanceRequest(True)
        )

        self.assertEqual(dis, -1.0)
        self.assertEqual(result.min_distance, -1.0)

        '''
        # fails if collided? values are very far from 0.0
        for i in result.nearest_points:
            for j in i:
                self.assertAlmostEqual(0.0, j)
        '''

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

        ret, result = fcl.collide(collision_object, p, cd.CollisionRequest())
        # Warning: collision function between node type 9 and node type 17 is not supported
        self.assertTrue(ret==0)

    def test_broadphase(self):
        objs = [fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0)),
                fcl.CollisionObject(fcl.Sphere(4.0)),
                fcl.CollisionObject(fcl.Cone(5.0, 6.0))]

        cyl = fcl.CollisionObject(fcl.Cylinder(7.0, 8.0))

        # Register objects to DynamicAABBTreeCollisionManager
        manager = fcl.DynamicAABBTreeCollisionManager()
        self.assertEqual(manager.size(), 0)
        manager.registerObjects(objs)
        self.assertEqual(manager.size(), 3)
        manager.registerObject(cyl)
        self.assertEqual(manager.size(), 4)

        # Use Callback function
        def cb_func(obj1, obj2, res):
            print("collide_callback_func start")
            ret, res = fcl.collide(obj1, obj2, fcl.CollisionRequest())
            print("result: ", ret)
            return ret

        manager.setup()

        res = fcl.CollisionResult()
        manager.collide(res, cb_func)
        manager.collide(objs[0], objs[1], cb_func)
        print("res?", res.contacts)

        # Initialize the collision manager for the first group of objects.
        # FCL provides various different implementations of CollisionManager.
        # Generally, the DynamicAABBTreeCollisionManager would provide the best performance.
        manager1 = fcl.DynamicAABBTreeCollisionManager()
        # Initialize the collision manager for the second group of objects.
        manager2 = fcl.DynamicAABBTreeCollisionManager()
        # To add objects into the collision manager, using BroadPhaseCollisionManager::registerObject() function to add one object
        # std::vector<CollisionObject*> objects1 = ...
        # for(std::size_t i = 0; i < objects1.size(); ++i)
        # manager1->registerObject(objects1[i]);

        manager1.registerObjects(objs)



        # Another choose is to use  BroadPhaseCollisionManager::registerObjects() function to add a set of objects
        # std::vector<CollisionObject*> objects2 = ...
        # manager2->registerObjects(objects2);

        manager2.registerObject(cyl)

        # In order to collect the information during broadphase, CollisionManager requires two settings:
        # a) a callback to collision or distance;
        # b) an intermediate data to store the information generated during the broadphase computation
        # For a), FCL provides the default callbacks for both collision and distance.
        # For b), FCL uses the CollisionData structure for collision and DistanceData structure for distance. CollisionData/DistanceData is just a container including both the CollisionRequest/DistanceRequest and CollisionResult/DistanceResult structures mentioned above.
        # CollisionData collision_data;
        # DistanceData distance_data;
        # Setup the managers, which is related with initializing the broadphase acceleration structure according to objects input
        # manager1->setup();
        # manager2->setup();

        manager1.setup()
        manager2.setup()

        # Examples for various queries
        # 1. Collision query between two object groups and get collision numbers
        # manager2->collide(manager1, &collision_data, defaultCollisionFunction);
        from fcl import  collision_data as cd
        res = cd.CollisionResult()
        # manager2.collide(manager1, res, cb_func)

        # manager2.collide(manager1, res)

        print("xxx")
        manager2.collide(res, cb_func)
        # int n_contact_num = collision_data.result.numContacts();
        # 2. Distance query between two object groups and get the minimum distance
        # manager2->distance(manager1, &distance_data, defaultDistanceFunction);
        # double min_distance = distance_data.result.min_distance;
        # 3. Self collision query for group 1
        # manager1->collide(&collision_data, defaultCollisionFunction);
        # 4. Self distance query for group 1
        # manager1->distance(&distance_data, defaultDistanceFunction);
        # 5. Collision query between one object in group 1 and the entire group 2
        # manager2->collide(objects1[0], &collision_data, defaultCollisionFunction);
        # 6. Distance query between one object in group 1 and the entire group 2
        # manager2->distance(objects1[0], &distance_data, defaultDistanceFunction);

