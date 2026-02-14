package com.aembot.lib.config.sensors.timeOfFlight;

import com.aembot.lib.core.can.CANDeviceID;

/**
 * Config for a time-of-flight sensor such as a CANRange.
 *
 * @see {@link com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO}
 */
public class TimeOfFlightConfiguration {
  public final String kName;

  /**
   * The CAN device associated with the configured time of flight sensor. Nullable depending on the
   * specific sensor.
   *
   * <p><strong>Default</strong>: null
   */
  public CANDeviceID kCANDeviceID;

  /**
   * The maximum distance from the sensor for an object to be detected.
   *
   * <p><strong>Default</strong>: -1
   */
  public double kDetectionThresholdMeters = -1;

  // Docs stolen from CTRE :p
  /**
   * How far above and below the threshold the distance needs to be to trigger undetected and
   * detected, respectively. This is used to prevent bouncing between the detected and undetected
   * states for objects on the threshold.
   *
   * <p>If the threshold is set to 0.1 meters, and the hysteresis is 0.01 meters, then an object
   * needs to be within 0.09 meters to be detected. After the object is first detected, the distance
   * then needs to exceed 0.11 meters to become undetected again.
   *
   * <p><strong>Default</strong>: 0.01
   */
  public double kDetectionHysteresisMeters = 0.01;

  public TimeOfFlightConfiguration(String name) {
    this.kName = name;
  }

  /**
   * Set the CAN device associated with the configured time of flight sensor. Nullable depending on
   * the specific sensor.
   *
   * <p><strong>Default</strong>: null
   *
   * @return This {@link TimeOfFlightConfiguration} for chaining
   */
  public TimeOfFlightConfiguration withCANDeviceID(CANDeviceID id) {
    this.kCANDeviceID = id;
    return this;
  }

  /**
   * Set the maximum distance from the sensor for an object to be detected.
   *
   * <p><strong>Default</strong>: -1
   *
   * @return This {@link TimeOfFlightConfiguration} for chaining
   */
  public TimeOfFlightConfiguration withDetectionThresholdMeters(double threshold) {
    this.kDetectionThresholdMeters = threshold;
    return this;
  }

  /**
   * Set how far above and below the threshold the distance needs to be to trigger undetected and
   * detected, respectively. This is used to prevent bouncing between the detected and undetected
   * states for objects on the threshold.
   *
   * <p>If the threshold is set to 0.1 meters, and the hysteresis is 0.01 meters, then an object
   * needs to be within 0.09 meters to be detected. After the object is first detected, the distance
   * then needs to exceed 0.11 meters to become undetected again.
   *
   * <p><strong>Default</strong>: 0.01
   *
   * @return This {@link TimeOfFlightConfiguration} for chaining
   */
  public TimeOfFlightConfiguration withDetectionHysteresisMeters(double hysteresis) {
    this.kDetectionHysteresisMeters = hysteresis;
    return this;
  }

  /**
   * Check that all values required for a time of flight sensor are set on this config. If they are
   * not, throw a {@link VerifyError}. Intended to be called at the end of an initialization chain.
   *
   * @return this {@link TimeOfFlightConfiguration} for chaining
   */
  public TimeOfFlightConfiguration validate() {
    // Not needed for this config type but I think it's good to have it just for consistency
    return this;
  }
}
