package com.aembot.frc2026.subsystems.indexerKicker.io;

import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.core.tracing.Traced;

public class IndexerKickerMechanismIOReplay implements IndexerKickerMechanismIO {
  private final MotorIO io = new MotorIOReplay();

  @Override
  public MotorIO getMotor() {
    return io;
  }

  @Override
  @Traced
  public void updateInputs(IndexerKickerMechanismInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
