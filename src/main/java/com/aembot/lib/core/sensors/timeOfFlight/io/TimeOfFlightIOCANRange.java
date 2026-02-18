package com.aembot.lib.core.sensors.timeOfFlight.io;

import com.aembot.lib.config.sensors.timeOfFlight.CANRangeTimeOfFlightConfiguration;
import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.CANStatusLogger;
import com.aembot.lib.core.can.interfaces.CANable;
import com.aembot.lib.core.phoenix6.CTREUtil;
import com.aembot.lib.core.sensors.timeOfFlight.TimeOfFlightInputs;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class TimeOfFlightIOCANRange implements TimeOfFlightIO, CANable {
  protected final TimeOfFlightConfiguration kConfig;

  protected final CANrange kCanRange;
  protected final CANDeviceID kId;

  protected final StatusSignal<Distance> kDistanceSignal;
  protected final StatusSignal<Distance> kStdDevsSignal;

  protected final List<BaseStatusSignal> kStatusSignals;

  public TimeOfFlightIOCANRange(TimeOfFlightConfiguration config) {
    this.kConfig = config;
    this.kId = config.kCANDeviceID;
    this.kCanRange = new CANrange(kId.getDeviceID(), kId.getBus());

    kId.setStatusSignal(kCanRange.getSupplyVoltage());

    kDistanceSignal = kCanRange.getDistance();
    kStdDevsSignal = kCanRange.getDistanceStdDev();

    kStatusSignals = List.of(kDistanceSignal, kStdDevsSignal);

    CTREUtil.setUpdateFrequencyForAll(
        50.0, kStatusSignals.toArray(new BaseStatusSignal[0]), kId.getDeviceID());

    CTREUtil.Configuration.Sensors.TimeOfFlight.optimizeBusUtilization(kCanRange);

    CANStatusLogger.get(kId.getBusName()).registerCANDevice(kId);
  }

  public TimeOfFlightIOCANRange(CANRangeTimeOfFlightConfiguration config) {
    this((TimeOfFlightConfiguration) config);

    CTREUtil.Configuration.Sensors.TimeOfFlight.applyConfiguration(kCanRange, config);
  }

  @Override
  public CANDeviceID getCANDevice() {
    return kId;
  }

  @Override
  public void updateInputs(TimeOfFlightInputs inputs) {
    BaseStatusSignal.refreshAll(kStatusSignals);

    // Ignore values when the measurement is zero, as it's invalid. This is mostly to patch an issue
    // in sim.
    if (kDistanceSignal.getValueAsDouble() != 0) {
      inputs.distanceMeters = kDistanceSignal.getValueAsDouble();
      inputs.distanceStdDevMeters = kStdDevsSignal.getValueAsDouble();
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public TimeOfFlightConfiguration getConfig() {
    return kConfig;
  }
}
