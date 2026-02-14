package com.aembot.lib.core.sensors.timeOfFlight.io;

import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightSimIO;
import com.ctre.phoenix6.sim.CANrangeSimState;
import org.littletonrobotics.junction.Logger;

public class TimeOfFlightSimIOCanRange extends TimeOfFlightIOCANRange implements TimeOfFlightSimIO {
  private final CANrangeSimState kSimState;

  private double simulatedDistance = Double.NaN;

  public TimeOfFlightSimIOCanRange(TimeOfFlightConfiguration config) {
    super(config);

    this.kSimState = kCanRange.getSimState();
  }

  @Override
  public void setSimulatedDistance(double distanceMeters) {
    kSimState.setDistance(distanceMeters);
    simulatedDistance = distanceMeters;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/Simulation/SimulatedDistance", simulatedDistance);
  }
}
