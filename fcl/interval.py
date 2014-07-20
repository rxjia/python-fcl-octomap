import numpy as np

class Interval:
    def __init__(self, left=None, right=None):
        if left is None:
            self.i = np.zeros(2)
        elif getattr(left, '__iter__', False):
            self.i = np.array(left[:2])
        else:
            self.i = np.zeros(2)
            self.i[0] = left
            if right is None:
                self.i[1] = left
            else:
                self.i[1] = right
        self.i.sort()

    def setValue(self, a, b=None):
        if getattr(left, '__iter__', False):
            self.i = np.array(a[:2])
        else:
            self.i[0] = a
            if b is None:
                self.i[1] = a
            else:
                self.i[1] = b
        self.i.sort()

    def __getitem__(self, idx):
        return self.i[idx]

    def __setitem__(self, idx, value):
        self.i[idx] = value

    def __iter__(self):
        for i in self.i:
            yield i

    def __eq__(self, other):
        return np.array_equal(other)

    def __add__(self, other):
        if isinstance(other, Interval):
            return Interval(self.i + other.i)
        else:
            return Interval(self.i[0] + other, self.i[1] + other)

    __radd__ = __add__
    __iadd__ = __add__

    def __sub__(self, other):
        if isinstance(other, Interval):
            return Interval(self.i[0] - other.i[1],
                            self.i[1] - other.i[0])
        else:
            return Interval(self.i[0] - other, self.i[1] - other)

    def __rsub__(self, other):
        return Interval(other - self.i[0], other - self.i[1])

    __isub__ = __sub__

    def __mul__(self, other):
        if isinstance(other, Interval):
            if other.i[0] >= 0:
                if self.i[0] >= 0:
                    return Interval(self.i[0] * other.i[0], self.i[1] * other.i[1])
                if self.i[1] <= 0:
                    return Interval(self.i[0] * other.i[1], self.i[1] * other.i[0])
                return Interval(self.i[0] * other.i[1], self.i[1] * other.i[1])
            if other.i[1] <= 0:
                if self.i[0] >= 0:
                    return Interval(self.i[1] * other.i[0], self.i[0] * other.i[1])
                if self.i[1] <= 0:
                    return Interval(self.i[1] * other.i[1], self.i[0] * other.i[0])
                return Interval(self.i[1] * other.i[0], self.i[0] * other.i[0])
   
            if self.i[0] >= 0:
                return Interval(self.i[1] * other.i[0], self.i[1] * other.i[1])

            if self.i[1] <= 0:
                return Interval(self.i[0] * other.i[1], self.i[0] * other.i[0])

            v00 = self.i[0] * other.i[0]
            v11 = self.i[1] * other.i[1]
            if v00 <= v11:
                v01 = self.i[0] * other.i[1]
                v10 = self.i[1] * other.i[0]
                if v01 < v10:
                    return Interval(v01, v11)
                return Interval(v10, v11)
        
            v01 = self.i[0] * other.i[1]
            v10 = self.i[1] * other.i[0]
            if v01 < v10:
                return Interval(v01, v00)
            return Interval(v10, v00)

        else:
            if other >= 0:
                return Interval(self.i * other)
            else:
                return Interval(self.i * other)

    __rmul__ = __mul__

    __imul__ = __mul__

    def __div__(self, other):
        if isinstance(other, Interval):
            return self * Interval(1.0 / other.i[1], 1.0 / other.i[0])
        else:
            return self * (1.0 / other)

    def __rdiv__(self, other):
        return Interval(other / self.i[1], other / self.i[0])

    __idiv__ = __div__

    def __pow__(self, other):
        ans = Interval(self)
        for _ in range(other - 1):
            ans *= self
        return ans

    def overlap(self, other):
        if self.i[1] < other.i[0]:
            return False
        if self.i[0] > other.i[1]:
            return False
        return True

    def intersect(self, other):
        if self.i[1] < other.i[0]:
            return False
        if self.i[0] > other.i[1]:
            return False
        if self.i[1] > other.i[1]:
            self.i[1] = other.i[1]
        if self.i[0] < other.i[0]:
            self.i[0] = other.i[0]
        return True

    def __neg__(self):
        return Interval(-self.i[1], -self.i[0])

    def getAbsLower(self):
        if self.i[0] >= 0:
            return self.i[0]
        if self.i[1] >= 0:
            return 0
        return -self.i[1]

    def getAbsUpper(self):
        if self.i.sum() >= 0:
            return self.i[1]
        return self.i[0]

    def contains(self, v):
        if v < self.i[0]:
            return False
        if v > self.i[1]:
            return False
        return True

    def bound(self, other):
        if isinstance(other, Interval):
            if other.i[0] < self.i[0]:
                self.i[0] = other.i[0]
            if other.i[1] > self.i[1]:
                self.i[1] = other.i[1]
            return self
        else:
            if other < self.i[0]:
                self.i[0] = other
            if other > self.i[1]:
                self.i[1] = other
            return self

    def __str__(self):
        return str(self.i)

    def center(self):
        return self.i.mean()

    def diameter(self):
        return self.i[1] - self.i[0]

def bound(i, other):
    res = Interval(i)
    if isinstance(other, Interval):
        if other.i[0] < res.i[0]:
            res.i[0] = other.i[0]
        if other.i[1] > res.i[1]:
            res.i[1] = other.i[1]
        return res
    else:
        if other < res.i[0]:
            res.i[0] = other
        if other > res.i[1]:
            res.i[1] = other
        return res

class TimeInterval:
    def __init__(self, l, r):
        self.setValue(l, r)
    def setValue(self, l, r):
        self.t = Interval(l, r)
        self.t2 = Interval(l * self.t[0], r * self.t[1])
        self.t3 = Interval(l * self.t2[0], r * self.t2[1])
        self.t4 = Interval(l * self.t3[0], r * self.t3[1])
        self.t5 = Interval(l * self.t4[0], r * self.t4[1])
        self.t6 = Interval(l * self.t5[0], r * self.t5[1])
