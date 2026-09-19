class _ScoredMission:
    def __init__(self, name, max_points, score_fn=None):
        self.name = name
        self.max_points = max_points
        self.score_fn = score_fn
        self.achieved = False

    def score(self, *args, **kwargs):
        if self.score_fn:
            self.achieved = bool(self.score_fn(*args, **kwargs))
        return self.max_points if self.achieved else 0

    def reset(self):
        self.achieved = False


class ScoringSystem:
    def __init__(self):
        self.missions = {}

    def add(self, name, max_points, score_fn=None):
        self.missions[name] = _ScoredMission(name, max_points, score_fn)
        return self

    def evaluate(self, *args, **kwargs):
        for m in self.missions.values():
            if not m.achieved:
                m.score(*args, **kwargs)
        return self.total_score()

    def total_score(self):
        return sum(m.max_points for m in self.missions.values() if m.achieved)

    def remaining(self):
        return [name for name, m in self.missions.items() if not m.achieved]

    def mark(self, name, achieved):
        if name in self.missions:
            self.missions[name].achieved = bool(achieved)

    def reset(self):
        for m in self.missions.values():
            m.reset()
