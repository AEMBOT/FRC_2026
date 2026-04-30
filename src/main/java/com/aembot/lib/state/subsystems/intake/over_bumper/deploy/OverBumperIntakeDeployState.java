package com.aembot.lib.state.subsystems.intake.over_bumper.deploy;

import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;

public class OverBumperIntakeDeployState implements Loggable {

  public double deployPositionUnits = 0;

  public boolean isDeployed = false;

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    AEMLogger.recordOutput(standardPrefix + "/deployPosition ", deployPositionUnits);
    AEMLogger.recordOutput(standardPrefix + "/isDeployed", isDeployed);
  }
}
