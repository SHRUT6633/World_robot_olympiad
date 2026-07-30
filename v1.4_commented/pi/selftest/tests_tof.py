from .runner import SelfTestRunner, TestResult


def register_tof_tests(runner: SelfTestRunner, sensors):
    # Register one self-test per ToF sensor in the sensors dict.
    # Each sensor is tested independently: initialise, take 5 readings,
    # average them, and verify the average is within [0, 5000] mm.
    #
    # sensors -- dict mapping sensor name (str) to a ToF sensor object
    #            implementing init() and read().

    for name, sensor in sensors.items():
        # Create a closure that captures the sensor name and object.
        def make_test(n, s):
            def test():
                if s is None:
                    return TestResult(f"tof_{n}").skipped("Not available")
                s.init()
                readings = []
                for _ in range(5):
                    val = s.read()
                    if val is not None:
                        readings.append(val)
                if not readings:
                    return TestResult(f"tof_{n}").failed("No valid readings")
                avg = sum(readings) / len(readings)
                if avg < 0 or avg > 5000:
                    return TestResult(f"tof_{n}").failed(
                        f"Out of range: {avg:.0f}mm"
                    )
                return TestResult(f"tof_{n}").passed(f"{avg:.0f}mm avg", data=avg)

            return test

        runner.add(f"tof_{name}", make_test(name, sensor))


def register_tof_cross_test(runner: SelfTestRunner, cross_verifier):
    # Register a cross-verification test that simply checks whether a
    # cross_verifier object has the _sensors attribute (indicating that
    # multiple ToF sensors were configured and can be compared against
    # each other).  This is a placeholder for a more advanced test.
    def test():
        if not hasattr(cross_verifier, "_sensors"):
            return TestResult("tof_cross").skipped("No ToF sensors")
        return TestResult("tof_cross").passed()

    runner.add("tof_cross_verify", test)
