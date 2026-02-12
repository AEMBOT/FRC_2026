package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.config.subsystems.intake.over_bumper.run.TalonFXOverBumperIntakeRunConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRunInputs;

/** Real implementation for over the bumper intake game piece intaking io */
public class TalonFXOverBumperIntakeRunHardwareIO implements OverBumperIntakeRunIO {

  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXOverBumperIntakeRunConfiguration config;

  /**
   * Construct a new real over the bumper intake game piece intake system io
   *
   * @param config configuration to use for this io
   */
  public TalonFXOverBumperIntakeRunHardwareIO(TalonFXOverBumperIntakeRunConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public void updateInputs(OverBumperIntakeRunInputs inputs) {
    updateLog();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
