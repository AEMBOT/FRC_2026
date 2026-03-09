package com.aembot.frc2026.subsystems.indexerKicker.io;

import com.aembot.frc2026.config.subsystems.indexerKicker.IndexerKickerConfiguration;
import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.tracing.Traced;

public class IndexerKickerMechanismIOReal implements IndexerKickerMechanismIO {
  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final IndexerKickerConfiguration config;

  /**
   * @param config Configuration to use for this io layer
   */
  public IndexerKickerMechanismIOReal(IndexerKickerConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  @Traced(category = "Indexer")
  public void updateInputs(IndexerKickerMechanismInputs inputs) {}

  @Override
  @Traced(category = "Indexer")
  public void updateLog(String standardPrefix, String inputs) {}
}
