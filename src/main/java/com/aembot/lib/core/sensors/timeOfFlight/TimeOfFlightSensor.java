package com.aembot.lib.core.sensors.timeOfFlight;

import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightSimIO;
import org.littletonrobotics.junction.Logger;

/**
 * Class to wrap a {@link TimeOfFlightIO}, providing methods for utilities such as proximity-based
 * object detection. {@link #update()} must be called periodically for this to function properly.
 */
public class TimeOfFlightSensor implements Loggable {
  public final TimeOfFlightIO kIo;
  public final TimeOfFlightInputs kInputs;
  public final TimeOfFlightConfiguration kConfig;

  public final String kStandardPrefix;
  public final String kInputPrefix;

  /** Whether or not an object is detected in front of the sensor */
  private boolean detectedState = false;

  /**
   * Construct a time of flight sensor. Uses the config attached to the IO for config
   *
   * @param io The IO layer for the the time of flight sensor
   * @param parentStandardPrefix The parent subsystem's standard logging prefix
   * @param parentInputPrefix The parent subsystem's input logging prefix
   */
  public TimeOfFlightSensor(
      TimeOfFlightIO io, String parentStandardPrefix, String parentInputPrefix) {
    this.kInputs = new TimeOfFlightInputs();
    this.kIo = io;

    this.kConfig = io.getConfig();

    this.kStandardPrefix = parentStandardPrefix + "/" + kConfig.kName;
    this.kInputPrefix = parentInputPrefix + "/" + kConfig.kName;
  }

  /** Update the state of the TOF sensor, the inputs of the IO layer, & log useful values */
  public void update() {
    kIo.updateInputs(kInputs);
    Logger.processInputs(kInputPrefix, kInputs);

    if (kConfig.kDetectionThresholdMeters > 0) {
      if (getDistanceMeters()
          < kConfig.kDetectionThresholdMeters - kConfig.kDetectionHysteresisMeters) {
        detectedState = true;
      } else if (getDistanceMeters()
          > kConfig.kDetectionThresholdMeters + kConfig.kDetectionHysteresisMeters) {
        detectedState = false;
      }
    }

    updateLog(kStandardPrefix, kInputPrefix);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/ObjectDetected", detectedState);
  }

  /** The distance measured by the TOF sensor in meters */
  public double getDistanceMeters() {
    return kInputs.distanceMeters;
  }

  /** Get the standard deviation of the distance measurement. -1.0 on init or if not implemented. */
  public double getDistanceStdDevMeters() {
    return kInputs.distanceStdDevMeters;
  }

  /**
   * Check if an object is detected given the detection threshold and hysteresis of the {@link
   * #kConfig}
   */
  public boolean getObjectDetected() {
    return detectedState;
  }

  /**
   * Set the simulated distance between the TOF sensor and a reflector in meters. No-op outside sim.
   */
  public void setSimulatedDistance(double distanceMeters) {
    if (kIo instanceof TimeOfFlightSimIO) {
      ((TimeOfFlightSimIO) kIo).setSimulatedDistance(distanceMeters);
    }
  }
}
