package com.aembot.lib.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.state.subsystems.flywheel.FlywheelState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.flywheel.io.FlywheelIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class FlywheelSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private final FlywheelIO flywheel;
  private final TalonFXFlywheelConfiguration config;

  /** Flywheel state instance to update */
  private final FlywheelState state;

  private double targetSpeed = 0;

  public FlywheelSubsystem(
      TalonFXFlywheelConfiguration config, FlywheelIO flywheel, FlywheelState stateInstance) {
    super(new MotorInputs(), flywheel.getMotor(), config.kMotorConfig);

    this.flywheel = flywheel;
    this.config = config;
    this.state = stateInstance;

    zeroEncoderPosition();
  }

  @Override
  public void periodic() {
    super.periodic();

    state.flywheelSpeedUnitsPerSecond.set(inputs.velocityUnitsPerSecond);
    state.atAcceptableSpeed.set(
        Math.abs(inputs.velocityUnitsPerSecond - targetSpeed)
            < config.kSpeedToleranceUnitsPerSecond);
  }

  @Override
  public void setSmartVelocitySetpointImpl(double velocity, int slot) {
    super.setSmartVelocitySetpointImpl(velocity, slot);
    targetSpeed = velocity;
  }

  @Override
  public void setPIDPositionSetpointImpl(double velocity, int slot) {
    super.setPIDPositionSetpointImpl(velocity, slot);
    targetSpeed = velocity;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    flywheel.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
