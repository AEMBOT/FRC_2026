package com.aembot.lib.core.sensors.timeOfFlight.interfaces;

import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.sensors.timeOfFlight.TimeOfFlightInputs;

public interface TimeOfFlightIO extends Loggable {
  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(TimeOfFlightInputs inputs);

  /** Get the config of this IO layer */
  public TimeOfFlightConfiguration getConfig();
}
