# v8.9 — Rate-Limited Error Logger

## What Changed

Our logging system was producing thousands of repetitive error messages, making it impossible to find actual problems in the logs. A single sensor dropout could generate 100 identical error messages per second:

```
[WARN] sensor_left: read timeout
[WARN] sensor_left: read timeout
[WARN] sensor_left: read timeout
[WARN] sensor_left: read timeout
... (repeats 10,000 times)
```

I built `error_logger.py` that rate-limits error messages and auto-disables sources that fail repeatedly. The key features:
- Rate limit: 1 message per 2 seconds per source
- Auto-disable: after 50 consecutive failures, the source is disabled
- Severity levels: CRITICAL always prints regardless of rate limit

## Errors Encountered

Wait — I was implementing this to fix errors, but I created a new one. After deploying the rate limiter, we missed an important error during the competition simulation:

```
[HEALTH_MONITOR] EMERGENCY STOP: Task 'control' heartbeat overdue
```

This message was logged exactly once. Then the rate limiter suppressed subsequent occurrences. The operator didn't see the repeated warnings and didn't realize the severity of the situation:

```
[ERROR_LOGGER] INFO: Suppressed 47 messages from 'health_monitor' in last 2.0s
```

The problem was my original design treated all messages the same — everything was rate-limited. Emergency stop messages (which repeat because the health monitor keeps checking) were being suppressed, and the operator had no idea the robot had stopped.

## The Fix

I added severity levels. Messages can be DEBUG, INFO, WARNING, ERROR, or CRITICAL. CRITICAL messages bypass the rate limiter entirely and are always printed. WARNING and below are rate-limited. ERROR is rate-limited but with a lower threshold (1 per 5 seconds).

```python
def log(self, source, message, severity=Severity.ERROR):
    if severity == Severity.CRITICAL:
        self._write(source, message, severity)  # always printed
        return
    # rate limit non-critical messages
    ...
```

I also added an auto-disable mechanism: if a source generates 50 consecutive failures, it's disabled until explicitly re-enabled. This prevents a stuck sensor from flooding the logs indefinitely.

## Alternatives Considered

1. **Circular buffer**: Keep the last N messages in a ring buffer and only print unique ones. This automatically rate-limits without explicit thresholds. But it requires storing all messages in memory, which is wasteful for a system that runs for hours.

2. **Exponential backoff**: Increase the suppression interval exponentially (2s, 4s, 8s, 16s...) for repeated messages from the same source. This naturally handles both burst errors (short backoff) and persistent errors (long backoff). But it makes it hard to guarantee that the operator sees at least one message per time period.

3. **Deduplication hash**: Store a hash of the last N messages and skip duplicates. This is simple but doesn't handle the case where the same error occurs intermittently (different timestamps would produce different hashes).

4. **External logging service**: Send all messages to a remote logging server that handles rate limiting and alerting. This would work for development but not competition (no network access on the track).

## Testing

- CRITICAL messages: always printed, zero suppression
- WARNING messages: max 1 per 2 seconds per source
- ERROR messages: max 1 per 5 seconds per source
- Auto-disable: source disabled after 50 consecutive failures
- Re-enable: source re-enabled after first non-failure call
- Memory usage: ~1KB per source, 10 sources = ~10KB total
- Performance: < 0.01ms overhead per log call

## Lessons Learned

Rate limiting is a double-edged sword. Done wrong, it hides important information. The key insight is that not all errors are equal — a CRITICAL error (emergency stop, motor failure, sensor dead) should never be silenced. I also learned that auto-disable needs a recovery mechanism — a source that was silenced might start working again, and we need to notice that. For v9.0, I want to add a "burst mode" that temporarily increases the rate limit when multiple distinct sources are failing simultaneously, as that indicates a systemic problem.
