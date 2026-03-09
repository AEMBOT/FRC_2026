package com.aembot.lib.subsystems.intake.over_bumper.deploy.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployInputs;
import com.aembot.lib.tracing.Traced;

public class OverBumperIntakeDeployReplayIO implements OverBumperIntakeDeployIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  @Traced(category = "Intake")
  public void updateInputs(OverBumperIntakeDeployInputs inputs) {}

  @Override
  @Traced(category = "Intake")
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
