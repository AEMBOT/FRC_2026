package com.aembot.lib.subsystems.intake.over_bumper.deploy.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployInputs;

public class OverBumperIntakeDeployReplayIO implements OverBumperIntakeDeployIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  public void updateInputs(OverBumperIntakeDeployInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
