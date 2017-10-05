## FCL-Python -- Python Interface for the Flexible Collision Library 

FCL-Python is an (unofficial) Python interface for the [Flexible Collision Library (FCL)](https://github.com/flexible-collision-library/fcl),
an excellent C++ library for performing proximity and collision queries on pairs of geometric models.
Currently, this package is targeted to FCL 0.5.0.

This package supports three types of proximity queries for pairs of geometric models:
* __Collision Detection__: Detecting whether two models overlap (and optionally where).
* __Distance Computation__: Computing the minimum distance between a pair of models.
* __Continuous Collision Detection__: Detecting whether two moving models overlap during movement (and optionally the time of contact).

This package also supports most of FCL's object shapes, including:
* TriangleP
* Box
* Sphere
* Ellipsoid
* Capsule
* Cone
* Cylinder
* Half-Space
* Plane
* Mesh

## Installation

First, install FCL using the instructions provided [here](https://github.com/flexible-collision-library/fcl).
If you're on Ubuntu 17.04 or newer, you can install FCL using `sudo apt-get install libfcl-dev`. This should also work on Ubuntu 16.04 and 16.10, but the default package for those versions is FCL 0.3.2-1.
Otherwise, just compile FCL from source -- it's quick and easy, and its dependencies are all easily installed via `apt`.

In order to install the Python wrappers for FCL, simply run
```shell
pip install python-fcl
```

## Objects
The interfaces to FCL are approximately the same as the ones provided by the C++ interface.

### Collision Objects
The primary construct in FCL is the `CollisionObject`, which forms the backbone of all collision and distance computations.
A `CollisionObject` consists of two components -- its geometry, defined by a `CollisionGeometry` object,
  and its pose, defined by a `Transform` object.

#### Collision Geometries
There are two main types of `CollisionGeometry` objects -- geometric primitives, such as boxes and spheres,
and arbitrary triangular meshes.
Here's some examples of how to instantiate geometric primitives.
Note that the box, sphere, ellipsoid, capsule, cone, and cylinder are all centered at the origin.

```python
import numpy as np
import fcl

v1 = np.array([1.0, 2.0, 3.0])
v2 = np.array([2.0, 1.0, 3.0])
v3 = np.array([3.0, 2.0, 1.0])
x, y, z = 1, 2, 3
rad, lz = 1.0, 3.0
n = np.array([1.0, 0.0, 0.0])
d = 5.0

t = fcl.TriangleP(v1, v2, v3) # Triangle defined by three points
b = fcl.Box(x, y, z)          # Axis-aligned box with given side lengths
s = fcl.Sphere(rad)           # Sphere with given radius
e = fcl.Ellipsoid(x, y, z)    # Axis-aligned ellipsoid with given radii
c = fcl.Capsule(rad, lz)      # Capsule with given radius and height along z-axis
c = fcl.Cone(rad, lz)         # Cone with given radius and cylinder height along z-axis
c = fcl.Cylinder(rad, lz)     # Cylinder with given radius and height along z-axis
h = fcl.Halfspace(n, d)       # Half-space defined by {x : <n, x> < d}
p = fcl.Plane(n, d)           # Plane defined by {x : <n, x> = d}
```

Triangular meshes are wrapped by the `BVHModel` class, and they are instantiated a bit differently.
```python
verts = np.array([[1.0, 1.0, 1.0],
                  [2.0, 1.0, 1.0],
                  [1.0, 2.0, 1.0],
                  [1.0, 1.0, 2.0]])
tris  = np.array([[0,2,1],
                  [0,3,2],
                  [0,1,3],
                  [1,2,3]])

m = fcl.BVHModel()
m.beginModel(len(verts), len(tris))
m.addSubModel(verts, tris)
m.endModel()
```

#### Transforms
In addition to a `CollisionGeometry`, a `CollisionObject` requires a `Transform`, which tells FCL where the `CollisionGeometry` is actually located in the world.
All `Transform` objects specify a rigid transformation (i.e. a rotation and a translation).
The translation is always a 3-entry vector, while the rotation can be specified by a 3x3 rotation matrix or a 4-entry quaternion.

Here are some examples of possible ways to instantiate and manipulate a `Transform`.

```python
R = np.array([[0.0, -1.0, 0.0],
              [1.0,  0.0, 0.0],
              [0.0,  0.0, 1.0]])
T = np.array([1.0, 2.0, 3.0])
q = np.array([0.707, 0.0, 0.0, 0.707])

tf = fcl.Transform()     # Default gives identity transform
tf = fcl.Transform(q)    # Quaternion rotation, zero translation
tf = fcl.Transform(R)    # Matrix rotation, zero translation
tf = fcl.Transform(T)    # Translation, identity rotation
tf = fcl.Transform(q, T) # Quaternion rotation and translation
tf = fcl.Transform(R, T) # Matrix rotation and translation
tf1 = fcl.Transform(tf)  # Can also initialize with another Transform
```

Now, given a `CollisionGeometry` and a `Transform`, we can create a `CollisionObject`:

```python
t = fcl.Transform(R, T)
b = fcl.Box(x, y, z)
obj = fcl.CollisionObject(b, t)
```

The transform of a collision object can be modified in-place:
```python
t1 = fcl.Transform(R1, T1)
obj.setTransform(t1)   # Using a transform
obj.setRotation(R2)    # Specifying components individually
obj.setTranslation(T2)
obj.setQuatRotation(q2)
```
## Commands

### Pairwise Operations

Given a pair of collision objects, this library supports three types of queries:
* __Collision Detection__: Detecting whether two models overlap (and optionally where).
* __Distance Computation__: Computing the minimum distance between a pair of models.
* __Continuous Collision Detection__: Detecting whether two moving models overlap during movement (and optionally the time of contact).

The interfaces for each of these operations follow a common pipeline.
First, a query request data structure is initialized and populated and an empty query response structure is initialized. Then, the query function is called and passed the two `CollisionObject` items and the request and response.
A scalar result is returned by the query function, and additional information is stored in the query result data structure.
Examples of all three operations are shown below.

#### Collision Checking

```python
g1 = fcl.Box(1,2,3)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)

g2 = fcl.Cone(1,3)
t2 = fcl.Transform()
o2 = fcl.CollisionObject(g2, t2)

request = fcl.CollisionRequest()
result = fcl.CollisionResult()

ret = fcl.collide(o1, o2, request, result)
```

After calling `fcl.collide()`, `ret` contains the number of contacts generated between the two objects,
and `result` contains information about the collision and contacts.
For more information about available parameters for collision requests and results,
see `fcl/collision_data.py`.

#### Distance Checking

```python
g1 = fcl.Box(1,2,3)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)

g2 = fcl.Cone(1,3)
t2 = fcl.Transform()
o2 = fcl.CollisionObject(g2, t2)

request = fcl.DistanceRequest()
result = fcl.DistanceResult()

ret = fcl.distance(o1, o2, request, result)
```

After calling `fcl.distance()`, `ret` contains the minimum distance between the two objects
and `result` contains information about the closest points on the objects.
For more information about available parameters for distance requests and results,
see `fcl/collision_data.py`.

#### Continuous Collision Checking

```python
g1 = fcl.Box(1,2,3)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)
t1_final = fcl.Transform(np.array([1.0, 0.0, 0.0]))

g2 = fcl.Cone(1,3)
t2 = fcl.Transform()
o2 = fcl.CollisionObject(g2, t2)
t2_final = fcl.Transform(np.array([-1.0, 0.0, 0.0]))

request = fcl.ContinuousCollisionRequest()
result = fcl.ContinuousCollisionResult()

ret = fcl.continuousCollide(o1, t1_final, o2, t2_final, request, result)
```

After calling `fcl.continuousCollide()`, `ret` contains the time of contact in (0,1), or 1.0 if the objects did not collide during movement from their initial poses to their final poses.
Additionally, `result` contains information about the collision time and status.
For more information about available parameters for continuous collision requests and results,
see `fcl/collision_data.py`.

### Broadphase Checking
In addition to pairwise checks, FCL supports broadphase collision/distance between groups of objects and can avoid the n square complexity.


For collision, the broadphase algorithm can return all collision pairs. For distance, it can return the pair with the minimum distance. FCL uses a CollisionManager structure to manage all the objects involving the collision or distance operations.

```cpp
// Initialize the collision manager for the first group of objects. 
// FCL provides various different implementations of CollisionManager.
// Generally, the DynamicAABBTreeCollisionManager would provide the best performance.
BroadPhaseCollisionManager* manager1 = new DynamicAABBTreeCollisionManager(); 
// Initialize the collision manager for the second group of objects.
BroadPhaseCollisionManager* manager2 = new DynamicAABBTreeCollisionManager();
// To add objects into the collision manager, using BroadPhaseCollisionManager::registerObject() function to add one object
std::vector<CollisionObject*> objects1 = ...
for(std::size_t i = 0; i < objects1.size(); ++i)
manager1->registerObject(objects1[i]);
// Another choose is to use BroadPhaseCollisionManager::registerObjects() function to add a set of objects
std::vector<CollisionObject*> objects2 = ...
manager2->registerObjects(objects2);
// In order to collect the information during broadphase, CollisionManager requires two settings: 
// a) a callback to collision or distance; 
// b) an intermediate data to store the information generated during the broadphase computation
// For a), FCL provides the default callbacks for both collision and distance.
// For b), FCL uses the CollisionData structure for collision and DistanceData structure for distance. CollisionData/DistanceData is just a container including both the CollisionRequest/DistanceRequest and CollisionResult/DistanceResult structures mentioned above.
CollisionData collision_data;
DistanceData distance_data;
// Setup the managers, which is related with initializing the broadphase acceleration structure according to objects input
manager1->setup();
manager2->setup();
// Examples for various queries
// 1. Collision query between two object groups and get collision numbers
manager2->collide(manager1, &collision_data, defaultCollisionFunction);
int n_contact_num = collision_data.result.numContacts(); 
// 2. Distance query between two object groups and get the minimum distance
manager2->distance(manager1, &distance_data, defaultDistanceFunction);
double min_distance = distance_data.result.min_distance;
// 3. Self collision query for group 1
manager1->collide(&collision_data, defaultCollisionFunction);
// 4. Self distance query for group 1
manager1->distance(&distance_data, defaultDistanceFunction);
// 5. Collision query between one object in group 1 and the entire group 2
manager2->collide(objects1[0], &collision_data, defaultCollisionFunction);
// 6. Distance query between one object in group 1 and the entire group 2
manager2->distance(objects1[0], &distance_data, defaultDistanceFunction); 
```
