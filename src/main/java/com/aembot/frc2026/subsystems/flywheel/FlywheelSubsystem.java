package com.aembot.frc2026.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorFollowerSubsystem;
import com.aembot.lib.subsystems.flywheel.FlywheelIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelSubsystem
    extends MotorFollowerSubsystem<
        MotorInputs, MotorIO, MotorFollowersConfiguration<TalonFXConfiguration>> {
  private final FlywheelIO flywheel;

  public FlywheelSubsystem(
      MotorFollowersConfiguration<TalonFXConfiguration> config, FlywheelIO flywheel) {

    super(
        new MotorInputs(),
        flywheel.getLeadMotor(),
        generateFollowerInputs(flywheel.getFollowerMotors()),
        flywheel.getFollowerMotors(),
        config);

    this.flywheel = flywheel;

    zeroEncoderPosition();
    setDefaultCommand(DefaultSpin());
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public Command holdPositionCommand() {
    return smartPositionSetpointCommand(this::getPositionSetpointUnits)
        .withName("HoldPosition")
        .ignoringDisable(true);
  }

  public Command DefaultSpin() {
    return smartVelocitySetpointCommand(() -> 40).withName("DefaultSpin").ignoringDisable(true);
  }
}
