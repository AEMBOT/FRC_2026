package com.aembot.lib.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.Logger;

/** Extension of the motor subsystem to add hood functionality */
public class HoodSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final HoodIO io;

  /** Configuration to use for this subsystem */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXHoodConfiguration config;

  /**
   * Construct a new hood subsystem
   *
   * @param config Configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   */
  public HoodSubsystem(TalonFXHoodConfiguration config, HoodIO io) {
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
