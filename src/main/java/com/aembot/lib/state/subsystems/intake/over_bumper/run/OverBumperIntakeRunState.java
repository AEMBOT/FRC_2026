package com.aembot.lib.state.subsystems.intake.over_bumper.run;

import com.aembot.lib.core.logging.Loggable;
import org.littletonrobotics.junction.Logger;

public class OverBumperIntakeRunState implements Loggable {

  public double angularVelocityUnitsPerMin;

  public boolean isActive;

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/angularVelocityUnitsPerMin", angularVelocityUnitsPerMin);
    Logger.recordOutput(standardPrefix + "/isActive", isActive);
  }
}
