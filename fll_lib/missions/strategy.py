class Mission:
    def __init__(self, name, points, duration_s=10, requires=None):
        self.name = name
        self.points = points
        self.duration_s = duration_s
        self.requires = requires or []

    def can_run(self, completed):
        return all(r in completed for r in self.requires)


class MissionPlan:
    def __init__(self, available_time_s=150):
        self.missions = []
        self.available_time_s = available_time_s

    def add(self, name, points, duration_s=10, requires=None):
        self.missions.append(Mission(name, points, duration_s, requires))
        return self

    def best_plan(self):
        selected = []
        completed = []
        total_time = 0
        total_points = 0
        sorted_m = sorted(
            self.missions,
            key=lambda m: (m.points / m.duration_s if m.duration_s else 0),
            reverse=True,
        )
        remaining = list(sorted_m)
        while remaining:
            progressed = False
            next_remaining = []
            for m in remaining:
                if m.duration_s <= 0 or total_time + m.duration_s > self.available_time_s:
                    continue
                if not m.can_run(completed):
                    next_remaining.append(m)
                    continue
                selected.append(m)
                completed.append(m.name)
                total_time += m.duration_s
                total_points += m.points
                progressed = True
            if not progressed:
                break
            remaining = next_remaining
        return selected, total_points

    def score(self, completed_names):
        return sum(m.points for m in self.missions if m.name in completed_names)

    def names(self):
        return [m.name for m in self.missions]
