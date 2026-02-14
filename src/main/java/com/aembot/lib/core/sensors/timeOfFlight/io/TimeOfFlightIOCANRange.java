package com.aembot.lib.core.sensors.timeOfFlight.io;

import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.interfaces.CANable;
import com.aembot.lib.core.sensors.timeOfFlight.TimeOfFlightInputs;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO;
import com.ctre.phoenix6.hardware.CANrange;

public class TimeOfFlightIOCANRange implements TimeOfFlightIO, CANable {
  protected final TimeOfFlightConfiguration kConfig;

  protected final CANrange kCanRange;
  protected final CANDeviceID kId;

  public TimeOfFlightIOCANRange(TimeOfFlightConfiguration config) {
    this.kConfig = config;
    this.kId = config.kCANDeviceID;
    this.kCanRange = new CANrange(kId.getDeviceID(), kId.getBus());

    kId.setStatusSignal(kCanRange.getSupplyVoltage());
  }

  @Override
  public CANDeviceID getCANDevice() {
    return kId;
  }

  @Override
  public void updateInputs(TimeOfFlightInputs inputs) {
    inputs.distanceMeters = kCanRange.getDistance().getValueAsDouble();
    inputs.distanceStdDevMeters = kCanRange.getDistanceStdDev().getValueAsDouble();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public TimeOfFlightConfiguration getConfig() {
    return kConfig;
  }
}
