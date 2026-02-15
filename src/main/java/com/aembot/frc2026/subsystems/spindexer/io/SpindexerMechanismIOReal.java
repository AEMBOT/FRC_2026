package com.aembot.frc2026.subsystems.spindexer.io;

import com.aembot.frc2026.config.subsystems.spindexer.SpindexerConfiguration;
import com.aembot.frc2026.subsystems.spindexer.SpindexerMechanismInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;

public class SpindexerMechanismIOReal implements SpindexerMechanismIO {
  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final SpindexerConfiguration config;

  /**
   * @param config Configuration to use for this io layer
   */
  public SpindexerMechanismIOReal(SpindexerConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public void updateInputs(SpindexerMechanismInputs inputs) {
    updateLog();
  }

  @Override
  public void updateLog(String standardPrefix, String inputs) {}
}
