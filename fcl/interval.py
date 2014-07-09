import numpy as np

class Interval:
    def __init__(self, left, right=None):
        if getattr(left, '__iter__', False):
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
        return Interval(self.i + other.i)

    def __sub__(self, other):
        return Interval(self.i[0] - other.i[1],
                        self.i[1] - other.i[0])

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

    def __div__(self, other):
        if isinstance(other, Interval):
            return self * Interval(1.0 / other.i[1], 1.0 / other.i[0])
        else:
            return self * (1.0 / other)

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
