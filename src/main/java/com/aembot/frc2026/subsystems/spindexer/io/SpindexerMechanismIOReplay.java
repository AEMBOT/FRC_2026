package com.aembot.frc2026.subsystems.spindexer.io;

import com.aembot.frc2026.subsystems.spindexer.SpindexerMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;

public class SpindexerMechanismIOReplay implements SpindexerMechanismIO {
  private final MotorIO io = new MotorIOReplay();

  @Override
  public MotorIO getMotor() {
    return io;
  }

  @Override
  public void updateInputs(SpindexerMechanismInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
