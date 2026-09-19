from fll_lib.utils import now_ms
from fll_lib.runtime import detect_platform


PLATFORM = detect_platform()


class Logger:
    def __init__(self, filepath="fll_log.txt", console=True):

        self.filepath = filepath if PLATFORM != "pybricks" else None
        self.console = console

    def log(self, msg):
        line = "[{}] {}".format(now_ms(), msg)
        if self.console:
            print(line)
        if self.filepath is None:
            return
        try:
            with open(self.filepath, "a") as f:
                f.write(line + "\n")
        except OSError:
            pass

    def clear(self):
        if self.filepath is None:
            return
        try:
            with open(self.filepath, "w") as f:
                f.write("")
        except OSError:
            pass
