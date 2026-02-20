package com.aembot.lib.core.sensors.timeOfFlight.io;

import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.sensors.timeOfFlight.TimeOfFlightInputs;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO;

public class TimeOfFlightIOReplay implements TimeOfFlightIO {
  private TimeOfFlightConfiguration kConfig;

  public TimeOfFlightIOReplay(TimeOfFlightConfiguration config) {
    this.kConfig = config;
  }

  @Override
  public void updateInputs(TimeOfFlightInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public TimeOfFlightConfiguration getConfig() {
    return kConfig;
  }
}
