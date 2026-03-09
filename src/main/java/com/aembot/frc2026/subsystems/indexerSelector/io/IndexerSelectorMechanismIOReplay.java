package com.aembot.frc2026.subsystems.indexerSelector.io;

import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.tracing.Traced;

public class IndexerSelectorMechanismIOReplay implements IndexerSelectorMechanismIO {
  private final MotorIO io = new MotorIOReplay();

  @Override
  public MotorIO getMotor() {
    return io;
  }

  @Override
  @Traced(category = "Indexer")
  public void updateInputs(IndexerSelectorMechanismInputs inputs) {}

  @Override
  @Traced(category = "Indexer")
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
