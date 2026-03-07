package com.aembot.frc2026.subsystems.indexerSelector.io;

import com.aembot.frc2026.config.subsystems.indexerSelector.IndexerSelectorConfiguration;
import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.core.tracing.Traced;

public class IndexerSelectorMechanismIOReal implements IndexerSelectorMechanismIO {
  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final IndexerSelectorConfiguration config;

  /**
   * @param config Configuration to use for this io layer
   */
  public IndexerSelectorMechanismIOReal(IndexerSelectorConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  @Traced
  public void updateInputs(IndexerSelectorMechanismInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputs) {}
}
