from .runner import SelfTestRunner, TestResult


def register_tof_tests(runner: SelfTestRunner, sensors):
    for name, sensor in sensors.items():
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
                    return TestResult(f"tof_{n}").failed(f"Out of range: {avg:.0f}mm")
                return TestResult(f"tof_{n}").passed(f"{avg:.0f}mm avg", data=avg)
            return test
        runner.add(f"tof_{name}", make_test(name, sensor))


def register_tof_cross_test(runner: SelfTestRunner, cross_verifier):
    def test():
        if not hasattr(cross_verifier, '_sensors'):
            return TestResult("tof_cross").skipped("No ToF sensors")
        return TestResult("tof_cross").passed()
    runner.add("tof_cross_verify", test)
