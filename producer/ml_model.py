import numpy as np

class OnlineAnomalyModel:
    def __init__(self):
        self.means = {}
        self.vars = {}
        self.count = 0

    def update(self, features):
        self.count += 1
        for k, v in features.items():
            v = float(v)

            if k not in self.means:
                self.means[k] = v
                self.vars[k] = 0
            else:
                delta = v - self.means[k]
                self.means[k] += delta / self.count
                self.vars[k] += delta * (v - self.means[k])

    def predict(self, features):
        score = 0
        total = 0

        for k, v in features.items():
            if k in self.means and self.count > 1:
                std = np.sqrt(self.vars[k] / (self.count - 1))
                if std > 0:
                    z = abs((float(v) - self.means[k]) / std)
                    score += z
                    total += 1

        return score / total if total > 0 else 0
