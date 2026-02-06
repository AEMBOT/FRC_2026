package com.aembot.lib.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  private final HoodInputs inputs;

  private final HoodIO io;

  private final TalonFXHoodConfiguration config;

  public HoodSubsystem(HoodInputs inputs, TalonFXHoodConfiguration config, HoodIO io) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.inputs = inputs;
    this.io = io;
    this.config = config;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateLog();
    super.periodic();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
