package com.aembot.lib.core.logging;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.core.logging.log_entries.LogEntry;
import java.util.HashMap;
import java.util.Map;

public final class AEMLogger {
  public static final Map<String, LogEntry<?>> keysToLogEntries = new HashMap<>();

  private static int staggerValue = 0;

  private static int i = 0;

  public static void addEntry(LogEntry<?> entry) {
    var previousValue = keysToLogEntries.putIfAbsent(entry.kKey, entry);
    if (previousValue != null && previousValue != entry)
      throw new IllegalArgumentException(
          "The given log entry's key has already been registered to a log entry.");
  }

  public static void tick() {
    i++;

    for (LogEntry<?> entry : keysToLogEntries.values()) {
      entry.tickSupplier();
      if ((RobotRuntimeConstants.MODE == RuntimeMode.REPLAY
              || (i + entry.kStagger) % entry.kThrottle == 0)
          && entry.hasBeenPushed()) {
        entry.recordOutput();
      }
    }
  }

  public static int getStagger(int throttle) {
    int val = staggerValue % (throttle - 1);
    staggerValue++;
    return val;
  }

  @SuppressWarnings("unchecked")
  public static <T> void recordOutput(String key, T value) {
    var entry = keysToLogEntries.get(key);
    if (entry == null) {
      entry = new LogEntry<>(key, value.getClass());
    } else if (value.getClass() != entry.kType) {
      throw new IllegalArgumentException(
          "The given log entry's key has already been registered to a log entry.");
    }

    ((LogEntry<T>) entry).pushValue(value);
  }
}
