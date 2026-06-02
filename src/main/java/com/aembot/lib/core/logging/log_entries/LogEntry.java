package com.aembot.lib.core.logging.log_entries;

import com.aembot.lib.core.logging.AEMLogger;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LogEntry<T> {
  /**
   * @see #kThrottle
   */
  public static final int DEFAULT_THROTTLE = 5;

  /** The key to log this entry at. */
  public final String kKey;

  /**
   * Nullable. Supplier of values for the entry. If non-null, it will be called periodically to
   * update the cached value of the entry.
   */
  public final Supplier<T> kSupplier;

  /**
   * The throttling applied to this log entry. The log entry's value will be published every n
   * frames. Set to 1 to disable throttling for this value. Throttling will always be disabled in
   * replay
   *
   * <p><strong>Default</strong>: {@value #DEFAULT_THROTTLE}
   */
  public final int kThrottle;

  /**
   * The stagger value for this log entry. An offset in periodic loop cycles for this entry to be
   * published at.
   */
  public final int kStagger;

  public final Class<T> kType;

  private final Consumer<T> kRecordOutputConsumer;

  private T cachedValue;
  private boolean pushed = false;
  private double pushTimestamp = -1;

  public LogEntry(String key, Class<T> type, int throttle, Supplier<T> supplier) {
    this.kKey = key;
    this.kSupplier = supplier;
    this.kThrottle = throttle;
    this.kType = type;

    this.kStagger = AEMLogger.getStagger(throttle);

    this.kRecordOutputConsumer = getLogConsumer(type);
    if (this.kRecordOutputConsumer == null)
      throw new IllegalArgumentException("The given type is not loggable.");

    AEMLogger.addEntry(this);
  }

  public LogEntry(String key, Class<T> type, Supplier<T> supplier) {
    this(key, type, DEFAULT_THROTTLE, supplier);
  }

  public LogEntry(String key, Class<T> type, int throttle) {
    this(key, type, throttle, null);
  }

  public LogEntry(String key, Class<T> type) {
    this(key, type, null);
  }

  public void tickSupplier() {
    if (kSupplier != null) {
      T val = kSupplier.get();
      if (val != null) pushValue(val);
    }
  }

  public void pushValue(T value) {
    this.cachedValue = value;
    this.pushTimestamp = Timer.getFPGATimestamp();
    this.pushed = true;
  }

  public boolean hasBeenPushed() {
    return this.pushed;
  }

  private Consumer<T> getLogConsumer(Class<T> type) {
    if (type == Double.class || type == double.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (double) val);
    } else if (type == Boolean.class || type == boolean.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (boolean) val);
    } else if (type == Integer.class || type == int.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (int) val);
    } else if (type == Long.class || type == long.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (long) val);
    } else if (type == Float.class || type == float.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (float) val);
    } else if (type == Character.class || type == char.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (char) val);
    } else if (type == Byte.class || type == byte.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (byte) val);
    } else if (type == Short.class || type == short.class) {
      return (T val) -> Logger.recordOutput(this.kKey, (short) val);
    }

    Method[] loggerMethods = Logger.class.getMethods();

    // methods that will work.
    List<Method> candidates = new ArrayList<>();

    for (Method method : loggerMethods) {
      if (method.getName().equals("recordOutput")
          && Modifier.isStatic(method.getModifiers())
          && method.getParameterCount() == 2
          && method.getParameterTypes()[0] == String.class) {

        // We wanna use StructSerializable if possible. A lot of classes will impl both
        // ProtobufSerializable & StructSerializable, and StructSerializable is generally better
        if (StructSerializable.class.isAssignableFrom(type)
            && StructSerializable.class.isAssignableFrom(method.getParameterTypes()[1])) {
          candidates.add(0, method);
          break;
        } else if (StructSerializable[].class.isAssignableFrom(type)
            && StructSerializable[].class.isAssignableFrom(method.getParameterTypes()[1])) {
          candidates.add(0, method);
          break;
        } else if (StructSerializable[][].class.isAssignableFrom(type)
            && StructSerializable[][].class.isAssignableFrom(method.getParameterTypes()[1])) {
          candidates.add(0, method);
          break;
        }

        if (method.getParameterTypes()[1].isAssignableFrom(type)) {
          candidates.add(method);
        }
      }
    }

    if (!candidates.isEmpty()) {
      Method method = candidates.get(0);
      return (T val) -> {
        try {
          method.invoke(null, this.kKey, val);
        } catch (IllegalAccessException | InvocationTargetException e) {
          throw new RuntimeException("Failed to invoke Logger.recordOutput", e);
        }
      };
    } else {
      return null;
    }
  }

  public T pullValue() {
    if (this.pushed) {
      this.pushed = false;
      return peekAtValue();
    } else {
      return null;
    }
  }

  public T peekAtValue() {
    return cachedValue;
  }

  public void recordOutput() {
    kRecordOutputConsumer.accept(cachedValue);
  }
}
