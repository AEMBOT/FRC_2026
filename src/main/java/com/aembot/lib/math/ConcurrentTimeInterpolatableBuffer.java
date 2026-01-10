package com.aembot.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.ConcurrentNavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

/**
 * **ConcurrentTimeInterpolatableBuffer: A Thread-Safe Data History for Robotics**
 *
 * <p>This class is designed to store a short history of data samples (like robot poses or sensor
 * readings) keyed by their precise **timestamp**, and it lets you **interpolate** to find the
 * estimated value at *any* time within that recorded history.
 *
 * <p>This is the **concurrent version** of the standard WPILib time buffer.
 *
 * <p>--- **Why use this? (The Concurrency Advantage)**
 *
 * <p>We use a {@link ConcurrentSkipListMap} to store the snapshots. This structure is ic nherently
 * **thread-safe** and **sorted by time**, which means you can have one thread (e.g., your
 * Drivetrain Odometry loop) constantly **writing** new data and another thread (e.g., your Vision
 * processing loop) simultaneously **reading** old data without needing to use explicit
 * `synchronized` blocks. This prevents timing issues and avoids slowing down your main robot loop.
 *
 * <p>--- **Primary Use Case: Pose Estimation**
 *
 * <p>When dealing with sensors that have **latency** (like a camera), you need to know what the
 * robot's state was *at the moment the sensor captured the data*, not when the data was processed.
 * This buffer is essential for:
 *
 * <p>1. **Time Alignment:** Storing a history of high-rate data (like poses). 2. **State
 * Reconstruction:** When a vision packet arrives with timestamp T, this buffer allows you to
 * calculate the robot's estimated pose at T, even if T falls between two recorded poses. 3.
 * **Memory Management:** Samples older than the configured {@code historySizeSeconds} are
 * automatically discarded to keep memory usage in check.
 *
 * @param <T> The type of data you're storing (e.g., a Pose2d, Rotation2d, or Double). The type must
 *     be able to calculate a value between two points (i.e., implement {@link Interpolatable} or be
 *     used with an external {@link Interpolator}).
 * @see <a
 *     href="https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/lib/util/ConcurrentTimeInterpolatableBuffer.java">Source
 *     Yoinked from Team 254 (The Cheesy Poofs)</a>
 */
public class ConcurrentTimeInterpolatableBuffer<T> {
  private final double historySize;
  private final Interpolator<T> interpolatingFunc;
  private final ConcurrentNavigableMap<Double, T> pastSnapshots = new ConcurrentSkipListMap<>();

  private ConcurrentTimeInterpolatableBuffer(
      Interpolator<T> interpolateFunction, double historySizeSeconds) {

    this.historySize = historySizeSeconds;
    this.interpolatingFunc = interpolateFunction;
  }

  /**
   * Create a new TimeInterpolatableBuffer.
   *
   * @param interpolateFunction The function used to interpolate between values.
   * @param historySizeSeconds The history size of the buffer.
   * @param <T> The type of data to store in the buffer.
   * @return The new TimeInterpolatableBuffer.
   */
  public static <T> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
      Interpolator<T> interpolateFunction, double historySizeSeconds) {

    return new ConcurrentTimeInterpolatableBuffer<>(interpolateFunction, historySizeSeconds);
  }

  /**
   * Create a new TimeInterpolatableBuffer.
   *
   * @param interpolateFunction The function used to interpolate between values.
   * @param historySizeSeconds The history size of the buffer.
   * @param <T> The type of data to store in the buffer.
   * @return The new TimeInterpolatableBuffer.
   */
  public static <T extends Interpolatable<T>> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
      double historySizeSeconds) {

    return new ConcurrentTimeInterpolatableBuffer<>(
        Interpolatable::interpolate, historySizeSeconds);
  }

  /**
   * create a new TimeInterpolatableBuffer to store Double values.
   *
   * @param historySizeSeconds the History size of the buffer.
   * @return The new TimeInterpolatableBuffer.
   */
  public static ConcurrentTimeInterpolatableBuffer<Double> createDoubleBuffer(
      double historySizeSeconds) {
    return new ConcurrentTimeInterpolatableBuffer<>(MathUtil::interpolate, historySizeSeconds);
  }

  /**
   * Add a sample to the buffer.
   *
   * @param timeSeconds The timestamp of the sample.
   * @param sample The sample object.
   */
  public void addSample(double timeSeconds, T sample) {
    pastSnapshots.put(timeSeconds, sample);
    cleanUp(timeSeconds);
  }

  /**
   * Removes samples older than the history size in reference to a given time.
   *
   * @param time The reference timestamp.
   */
  public void cleanUp(double time) {
    pastSnapshots.headMap(time - historySize, false).clear();
  }

  /** Clear all samples from the map. */
  public void clear() {
    pastSnapshots.clear();
  }

  /**
   * Sample the buffer at the given time. If the buffer is empty, an empty Optional is returned
   *
   * @param timeSeconds The time at which to sample.
   * @return The interpolated value at that time, or an empty Optional.
   */
  public Optional<T> getSample(double timeSeconds) {
    if (pastSnapshots.isEmpty()) {
      return Optional.empty();
    }

    // Case for if the requested time is exact same as a sample
    T nowEntry = pastSnapshots.get(timeSeconds);
    if (pastSnapshots.get(timeSeconds) != null) {
      return Optional.of(nowEntry);
    }

    Entry<Double, T> startEntry = pastSnapshots.floorEntry(timeSeconds);
    Entry<Double, T> endEntry = pastSnapshots.ceilingEntry(timeSeconds);

    // Check if either value is null, and return the other if it is
    // Occurs when asking for a timestamp outside the scope of the samples
    if (startEntry == null) {
      return Optional.of(endEntry.getValue());
    }
    if (endEntry == null) {
      return Optional.of(startEntry.getValue());
    }

    T startValue = startEntry.getValue();
    T endValue = endEntry.getValue();

    Double startTime = startEntry.getKey();
    Double endTime = endEntry.getKey();

    // Get the ratio between
    // (the time between the time to check and the time of the start sample)
    // divided by
    // (the time between the time of the end sample and the time of the start sample)
    Double interpolationTime = (timeSeconds - startTime) / (endTime - startTime);

    return Optional.of(interpolatingFunc.interpolate(startValue, endValue, interpolationTime));
  }

  /**
   * @return The latest sample in the buffer
   */
  public Entry<Double, T> getLatest() {
    return pastSnapshots.lastEntry();
  }

  /**
   * Used in Pose Estimation to replay odometry inputs
   * stored within this buffer
   * 
   * @return The internal buffer used to store samples
   */
  public ConcurrentNavigableMap<Double, T> getInternalBuffer() {
    return pastSnapshots;
  }
}
