package com.aembot.lib.subsystems.intake.generic.run.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.subsystems.intake.generic.run.IntakeRollerInputs;

public class IntakeRollerReplayIO implements IntakeRollerIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  public void updateInputs(IntakeRollerInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
