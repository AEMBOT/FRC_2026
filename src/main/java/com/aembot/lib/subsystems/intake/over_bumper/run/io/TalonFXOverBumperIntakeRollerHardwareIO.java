package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.config.subsystems.intake.overBumper.run.TalonFXOverBumperIntakeRollerConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerInputs;

/** Real implementation for over the bumper intake roller io */
public class TalonFXOverBumperIntakeRollerHardwareIO implements OverBumperIntakeRollerIO {

  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXOverBumperIntakeRollerConfiguration config;

  /**
   * Construct a new real over the bumper intake game piece intake system io
   *
   * @param config configuration to use for this io
   */
  public TalonFXOverBumperIntakeRollerHardwareIO(
      TalonFXOverBumperIntakeRollerConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kRealMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public void updateInputs(OverBumperIntakeRollerInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
