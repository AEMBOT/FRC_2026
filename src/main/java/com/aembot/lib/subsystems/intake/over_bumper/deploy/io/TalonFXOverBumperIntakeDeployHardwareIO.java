package com.aembot.lib.subsystems.intake.over_bumper.deploy.io;

import com.aembot.lib.config.subsystems.intake.over_bumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployInputs;

/** Real over the bumper intake deployment io */
public class TalonFXOverBumperIntakeDeployHardwareIO implements OverBumperIntakeDeployIO {

  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXOverBumperIntakeDeployConfiguration config;

  /**
   * Construct a new real over the bumper intake deployment io
   *
   * @param config configuration to use for this io
   */
  public TalonFXOverBumperIntakeDeployHardwareIO(
      TalonFXOverBumperIntakeDeployConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public void updateInputs(OverBumperIntakeDeployInputs inputs) {
    updateLog();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
