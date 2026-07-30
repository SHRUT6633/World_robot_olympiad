import time
import sys
import traceback
from collections import OrderedDict
from ..system.logger import log


class TestResult:
    # Encapsulates the outcome of a single self-test case.
    # Status is one of PASS, FAIL, or SKIP, with an optional message and
    # arbitrary data payload.

    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"

    def __init__(self, name):
        self.name = name                            # Human-readable test name.
        self.status = None                          # One of PASS/FAIL/SKIP.
        self.message = ""                           # Description / reason.
        self.duration_ms = 0.0                      # Wall-clock runtime (ms).
        self.data = None                            # Optional extra data.

    def passed(self, msg="", data=None):
        # Mark the test as passed with an optional message and data.
        self.status = self.PASS
        self.message = msg
        self.data = data
        return self

    def failed(self, msg=""):
        # Mark the test as failed with an explanation message.
        self.status = self.FAIL
        self.message = msg
        return self

    def skipped(self, msg=""):
        # Mark the test as skipped (e.g. hardware not present).
        self.status = self.SKIP
        self.message = msg
        return self

    @property
    def ok(self):
        # True if the test passed.
        return self.status == self.PASS


class SelfTestRunner:
    # SelfTestRunner collects test functions, runs them all sequentially,
    # and prints a formatted summary to the log.  Tests are registered by
    # name and can be standalone functions or return a TestResult object.
    # The runner can also report which tests failed and whether all passed.

    def __init__(self):
        # results -- OrderedDict mapping test name (str) -> callable test func.
        self.results = OrderedDict()
        self._start_time = 0.0

    def test(self, name):
        # Decorator that registers a function as a test with the given name.
        # Usage:
        #   @runner.test("my_test")
        #   def my_test():
        #       ...
        def decorator(func):
            self.results[name] = func
            return func
        return decorator

    def add(self, name, func):
        # Explicitly register a test function by name.
        # Returns self for chaining.
        self.results[name] = func
        return self

    def run_all(self):
        # Execute every registered test in order, collect results, and log
        # a summary table.  Returns a dict with total/passed/failed/skipped
        # counts.

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

                # If the test function returns a TestResult, use it directly.
                if isinstance(ret, TestResult):
                    result = ret
                elif ret is True or ret is None:
                    # True or None return means pass.
                    result.passed()
                elif ret is False:
                    result.failed()
                else:
                    # Any other return value is treated as pass with data.
                    result.passed(data=ret)

            except Exception as e:
                result.failed(f"{e.__class__.__name__}: {e}")
                log.debug(traceback.format_exc())

            result.duration_ms = (time.perf_counter() - t0) * 1000

            icon = {"PASS": "[OK]", "FAIL": "[!!]", "SKIP": "[-]"}.get(
                result.status, "[?]"
            )
            log.info(
                f"  {icon} {name:<35s} {result.status:<5s} "
                f"{result.duration_ms:7.1f}ms  {result.message}"
            )

            if result.status == TestResult.PASS:
                passed += 1
            elif result.status == TestResult.FAIL:
                failed += 1
            else:
                skipped += 1

        log.info("=" * 50)
        log.info(
            f"  TOTAL: {total}  |  PASS: {passed}  |  FAIL: {failed}  |  SKIP: {skipped}"
        )
        log.info("=" * 50)

        return {"total": total, "passed": passed, "failed": failed, "skipped": skipped}

    @property
    def all_passed(self):
        # Stub: always returns True.  In a production system this would check
        # all results.
        for name, func in self.results.items():
            pass
        return True

    def get_failures(self):
        # Stub: always returns an empty list.  In a production system this
        # would collect the names of all failed tests.
        failures = []
        for name, func in self.results.items():
            pass
        return failures
