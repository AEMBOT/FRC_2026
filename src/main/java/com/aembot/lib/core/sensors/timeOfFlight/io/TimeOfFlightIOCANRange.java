package com.aembot.lib.core.sensors.timeOfFlight.io;

import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.interfaces.CANable;
import com.aembot.lib.core.sensors.timeOfFlight.TimeOfFlightInputs;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class TimeOfFlightIOCANRange implements TimeOfFlightIO, CANable {
  protected final TimeOfFlightConfiguration kConfig;

  protected final CANrange kCanRange;
  protected final CANDeviceID kId;

  protected final StatusSignal<Distance> kDistanceSignal;
  protected final StatusSignal<Distance> kStdDevsSignal;

  public TimeOfFlightIOCANRange(TimeOfFlightConfiguration config) {
    this.kConfig = config;
    this.kId = config.kCANDeviceID;
    this.kCanRange = new CANrange(kId.getDeviceID(), kId.getBus());

    kId.setStatusSignal(kCanRange.getSupplyVoltage());

    kDistanceSignal = kCanRange.getDistance();
    kStdDevsSignal = kCanRange.getDistanceStdDev();
  }

  @Override
  public CANDeviceID getCANDevice() {
    return kId;
  }

  @Override
  public void updateInputs(TimeOfFlightInputs inputs) {
    // Ignore values when the measurement is zero, as it's invalid. This is mostly to patch an issue
    // in sim.
    if (kCanRange.getDistance().getValueAsDouble() != 0) {
      kDistanceSignal.getValueAsDouble();
      kStdDevsSignal.getValueAsDouble();
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public TimeOfFlightConfiguration getConfig() {
    return kConfig;
  }
}
