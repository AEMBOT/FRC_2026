package com.aembot.lib.core.sensors.timeOfFlight;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TimeOfFlightInputs implements LoggableInputs {
  /** The distance measured by the TOF sensor in meters */
  public double distanceMeters = Double.NaN;

  /** The standard deviation of the distance measurement. -1.0 on init or if not implemented. */
  public double distanceStdDevMeters = -1.0;

  @Override
  public void toLog(LogTable table) {
    table.put("DistanceMeters", distanceMeters);
    table.put("DistanceStdDev", distanceStdDevMeters);
  }

  @Override
  public void fromLog(LogTable table) {
    distanceMeters = table.get("DistanceMeters", distanceMeters);
    distanceStdDevMeters = table.get("DistanceStdDev", distanceStdDevMeters);
  }
}
