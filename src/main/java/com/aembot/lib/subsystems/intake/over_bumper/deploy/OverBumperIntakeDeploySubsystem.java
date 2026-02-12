package com.aembot.lib.subsystems.intake.over_bumper.deploy;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.intake.over_bumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.OverBumperIntakeDeployIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.Logger;

/** Extension of the motor subsystem to add over the bumper intake deployment functionality */
public class OverBumperIntakeDeploySubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final OverBumperIntakeDeployIO io;

  /** Configuration to use for this subsytem */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXOverBumperIntakeDeployConfiguration config;

  /**
   * Construct a new over the bumper intake deployment subsystem
   *
   * @param config Configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   */
  public OverBumperIntakeDeploySubsystem(
      TalonFXOverBumperIntakeDeployConfiguration config, OverBumperIntakeDeployIO io) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.io = io;
    this.config = config;
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
