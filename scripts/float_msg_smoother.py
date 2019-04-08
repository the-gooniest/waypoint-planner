import numpy as np

class FloatMsgSmoother:

    def __init__(self, size, default, weight=None):
        self.values = []
        self.size = size
        self.default = default

        self.weight = weight
        if self.weight is not None:
            if weight < 0:
                self.weight = 0
            elif weight > 1:
                self.weight = 1

    def add_value(self, new_value):
        while len(self.values) > self.size:
            self.values.pop(0)
        self.values.append(new_value)

    def value(self):
        if len(self.values) == 0:
            new_value = self.default 
        elif len(self.values) == 1:
            new_value = self.values[-1]
        elif self.weight is None:
            new_value = np.mean(self.values)
        else:
            other_avg = np.mean(self.values[:-1])
            new_value = other_avg * (1 - self.weight) + self.weight * self.values[-1]
        return new_value