class DriftCompensator:
    def __init__(self, forward=1.0, backward=1.0):
        self.forward_factor = forward
        self.backward_factor = backward

    def compensate(self, distance_cm, direction="forward"):
        factor = self.forward_factor if direction == "forward" else self.backward_factor
        if factor <= 0:
            return distance_cm
        return distance_cm / factor

    @staticmethod
    def compute_factor(nominal_cm, measured_cm):
        if measured_cm <= 0:
            return 1.0
        return measured_cm / nominal_cm
