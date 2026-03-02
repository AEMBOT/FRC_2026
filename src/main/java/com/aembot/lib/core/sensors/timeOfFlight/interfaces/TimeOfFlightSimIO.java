package com.aembot.lib.core.sensors.timeOfFlight.interfaces;

public interface TimeOfFlightSimIO extends TimeOfFlightIO {
  /**
   * Set the simulated distance of the time of flight sensor
   *
   * @param distanceMeters The simulated distance detected by the sensor in meters
   */
  public void setSimulatedDistance(double distanceMeters);
}
