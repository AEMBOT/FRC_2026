package com.aembot.lib.subsystems.intake.over_bumper.deploy.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployInputs;

public class OverBumperIntakeDeployReplayIO implements OverBumperIntakeDeployIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  @Traced
  public void updateInputs(OverBumperIntakeDeployInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
