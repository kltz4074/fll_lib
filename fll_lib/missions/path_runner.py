from fll_lib.utils import sleep_ms


class PathRunner:
    def __init__(self, robot):
        self.robot = robot
        self.commands = []

    def add(self, cmd_type, *args):
        self.commands.append((cmd_type, args))
        return self

    def forward(self, cm, speed=60):
        return self.add("forward", cm, speed)

    def backward(self, cm, speed=60):
        return self.add("backward", cm, speed)

    def turn_right(self, angle, speed=40):
        return self.add("turn_right", angle, speed)

    def turn_left(self, angle, speed=40):
        return self.add("turn_left", angle, speed)

    def manip_l(self, angle, speed=50):
        return self.add("manip_l", angle, speed)

    def manip_r(self, angle, speed=50):
        return self.add("manip_r", angle, speed)

    def both_manips(self, angle_l, angle_r, speed=100):
        return self.add("both_manips", angle_l, angle_r, speed)

    def pause(self, ms):
        return self.add("pause", ms)

    def reset_gyro(self):
        return self.add("reset_gyro")

    def run(self):
        for cmd_type, args in self.commands:
            if cmd_type == "forward":
                self.robot.forward(args[0], args[1])
            elif cmd_type == "backward":
                self.robot.backward(args[0], args[1])
            elif cmd_type == "turn_right":
                self.robot.turn_right(args[0], args[1])
            elif cmd_type == "turn_left":
                self.robot.turn_left(args[0], args[1])
            elif cmd_type == "manip_l":
                self.robot.move_manip_l(args[0], args[1])
            elif cmd_type == "manip_r":
                self.robot.move_manip_r(args[0], args[1])
            elif cmd_type == "both_manips":
                self.robot.move_both_manipulators(args[0], args[1], args[2])
            elif cmd_type == "pause":
                sleep_ms(args[0])
            elif cmd_type == "reset_gyro":
                self.robot.reset_gyro()
