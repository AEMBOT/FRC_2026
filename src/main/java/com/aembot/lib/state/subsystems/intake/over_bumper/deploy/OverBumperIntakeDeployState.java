package com.aembot.lib.state.subsystems.intake.over_bumper.deploy;

import com.aembot.lib.core.logging.Loggable;
import org.littletonrobotics.junction.Logger;

public class OverBumperIntakeDeployState implements Loggable {

  public double deployPositionUnits = 0;

  public boolean isDeployed = false;

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/deployPosition ", deployPositionUnits);
    Logger.recordOutput(standardPrefix + "/isDeployed", isDeployed);
  }
}
