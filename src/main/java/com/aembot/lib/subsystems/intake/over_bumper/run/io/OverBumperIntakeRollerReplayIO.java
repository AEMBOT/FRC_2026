package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerInputs;

public class OverBumperIntakeRollerReplayIO implements OverBumperIntakeRollerIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  @Traced
  public void updateInputs(OverBumperIntakeRollerInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
