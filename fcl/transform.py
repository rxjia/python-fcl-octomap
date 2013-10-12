import numpy as np

def rotation_to_quaternion(rot):
    q = np.array(4)
    q[0] = np.sqrt(max((rot[0, 0] + rot[1, 1] + rot[2, 2] + 1.0)/4.0, 0.0))
    q[1] = np.sqrt(max((rot[0, 0] - rot[1, 1] - rot[2, 2] + 1.0)/4.0, 0.0))
    q[2] = np.sqrt(max((-rot[0, 0] + rot[1, 1] - rot[2, 2] + 1.0)/4.0, 0.0))
    q[3] = np.sqrt(max((-rot[0, 0] - rot[1, 1] + rot[2, 2] + 1.0)/4.0, 0.0))
    if q[0] >= q[1] and q[0] >= q[2] and q[0] >= q[3]:
        q[1] *= np.sign(rot[2, 1] - rot[1, 2])
        q[2] *= np.sign(rot[0, 2] - rot[2, 0])
        q[3] *= np.sign(rot[1, 0] - rot[0, 1])
    elif q[1] >= q[0] and q[1] >= q[2] and q[1] >= q[3]:
        q[0] *= np.sign(rot[2, 1] - rot[1, 2])
        q[2] *= np.sign(rot[1, 0] + rot[0, 1])
        q[3] *= np.sign(rot[0, 2] + rot[2, 0])
    elif q[2] >= q[0] and q[2] >= q[1] and q[2] >= q[3]:
        q[0] *= np.sign(rot[0, 2] - rot[2, 0])
        q[1] *= np.sign(rot[1, 0] + rot[0, 1])
        q[3] *= np.sign(rot[2, 1] + rot[1, 2])
    elif q[3] >= q[0] and q[3] >= q[1] and q[3] >= q[2]:
        q[0] *= np.sign(rot[1, 0] - rot[0, 1])
        q[1] *= np.sign(rot[2, 0] + rot[0, 2])
        q[2] *= np.sign(rot[2, 1] + rot[1, 2])
    else:
        raise ValueError
    return q/np.linalg.norm(q)

class Quaternion(object):
    def __init__(self, *args):
        if len(args) == 0:
            self._data = np.zeros(4)
            self._data[0] = 1.0
        elif len(args) == 4:
            self._data = np.array(args)
        elif len(args) == 1 and len(args[0]) == 4:
            self._data = np.array(args[0])

    @property
    def w(self):
        return self._data[0]
    @w.setter
    def w(self, value):
        self._data[0] = value

    @property
    def x(self):
        return self._data[1]
    @x.setter
    def x(self, value):
        self._data[1] = value

    @property
    def y(self):
        return self._data[2]
    @y.setter
    def y(self, value):
        self._data[2] = value

    @property
    def z(self):
        return self._data[3]
    @z.setter
    def z(self, value):
        self._data[3] = value

    @property
    def v(self):
        return self._data[1:]
    @v.setter
    def v(self, value):
        self._data[1:] = value

    def __getitem__(self, idx):
        return self._data[idx]
    def __setitem__(self, idx, value):
        self._data[idx] = value

    def isIdentity(self):
        return self._data[0] == 1 and \
               all((d == 0 for d in self._data[1:]))
    def __add__(self, other):
        return Quaternion(self._data + other._data)
    def __sub__(self, other):
        return Quaternion(self._data - other._data)
    def __neg__(self):
        return Quaternion(-self._data)
    def __mul__(self, other):
        v = self.w * other.v + other.w * self.v + np.cross(self.v, other.v)
        return Quaternion(self.w * other.w - np.dot(self.v, other.v),
                          v[0], v[1], v[2])
    def conj(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __str__(self):
        return str(self._data)

class Transform:
    def __init__(self, rot=None, pos=None):
        if not pos is None:
            self.t = np.array(pos)
        else:
            self.t = np.zeros(3)

        if not rot is None:
            self.q = Quaternion()
        elif isinstance(rot ,Quaternion):
            self.q = rot
        else:
            self.q = rotation_to_quaternion(rot)
