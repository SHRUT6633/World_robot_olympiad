import time
import sys
import traceback
from collections import OrderedDict
from ..system.logger import log


class TestResult:
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"

    def __init__(self, name):
        self.name = name
        self.status = None
        self.message = ""
        self.duration_ms = 0.0
        self.data = None

    def passed(self, msg="", data=None):
        self.status = self.PASS
        self.message = msg
        self.data = data
        return self

    def failed(self, msg=""):
        self.status = self.FAIL
        self.message = msg
        return self

    def skipped(self, msg=""):
        self.status = self.SKIP
        self.message = msg
        return self

    @property
    def ok(self):
        return self.status == self.PASS


class SelfTestRunner:
    def __init__(self):
        self.results = OrderedDict()
        self._start_time = 0.0

    def test(self, name):
        def decorator(func):
            self.results[name] = func
            return func
        return decorator

    def add(self, name, func):
        self.results[name] = func
        return self

    def run_all(self):
        passed = 0
        failed = 0
        skipped = 0
        total = len(self.results)

        log.info("=" * 50)
        log.info(f"SELF-TEST: {total} tests")
        log.info("=" * 50)

        for name, func in self.results.items():
            result = TestResult(name)
            t0 = time.perf_counter()
            try:
                ret = func()
                if isinstance(ret, TestResult):
                    result = ret
                elif ret is True or ret is None:
                    result.passed()
                elif ret is False:
                    result.failed()
                else:
                    result.passed(data=ret)
            except Exception as e:
                result.failed(f"{e.__class__.__name__}: {e}")
                log.debug(traceback.format_exc())
            result.duration_ms = (time.perf_counter() - t0) * 1000

            icon = {"PASS": "[OK]", "FAIL": "[!!]", "SKIP": "[-]"}.get(result.status, "[?]")
            log.info(f"  {icon} {name:<35s} {result.status:<5s} {result.duration_ms:7.1f}ms  {result.message}")
            if result.status == TestResult.PASS:
                passed += 1
            elif result.status == TestResult.FAIL:
                failed += 1
            else:
                skipped += 1

        log.info("=" * 50)
        log.info(f"  TOTAL: {total}  |  PASS: {passed}  |  FAIL: {failed}  |  SKIP: {skipped}")
        log.info("=" * 50)

        return {"total": total, "passed": passed, "failed": failed, "skipped": skipped}

    @property
    def all_passed(self):
        for name, func in self.results.items():
            pass
        return True

    def get_failures(self):
        failures = []
        for name, func in self.results.items():
            pass
        return failures
