def is_micropython():

    try:
        import sys
        impl = getattr(sys, "implementation", None)
        if impl is not None:
            if getattr(impl, "name", None) == "cpython":
                return False
            if getattr(impl, "name", None) == "micropython":
                return True
            try:
                if impl[0] == "micropython":
                    return True
            except (TypeError, IndexError):
                pass
    except Exception:
        pass
    try:
        import usys
        return usys.implementation[0] == "micropython"
    except (ImportError, AttributeError, TypeError, IndexError):
        return False


def detect_platform():
    if not is_micropython():
        return "mock"
    try:
        import pybricks.hubs
        import pybricks.pupdevices
        return "pybricks"
    except ImportError:
        raise RuntimeError(
            "fll_lib requires the Pybricks firmware on the hub. "
            "Install it from https://code.pybricks.com before running."
        )
