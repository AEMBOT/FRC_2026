package com.aembot.lib.state.subsystems.intake.over_bumper.run;

import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;

public class OverBumperIntakeRollerState implements Loggable {

  public double angularVelocityUnitsPerMin;

  public boolean isActive;

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    AEMLogger.recordOutput(
        standardPrefix + "/angularVelocityUnitsPerMin", angularVelocityUnitsPerMin);
    AEMLogger.recordOutput(standardPrefix + "/isActive", isActive);
  }
}
