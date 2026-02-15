package com.aembot.frc2026.subsystems.indexerKicker.io;

import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;

public class IndexerKickerMechanismIOReplay implements IndexerKickerMechanismIO {
  private final MotorIO io = new MotorIOReplay();

  @Override
  public MotorIO getMotor() {
    return io;
  }

  @Override
  public void updateInputs(IndexerKickerMechanismInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
