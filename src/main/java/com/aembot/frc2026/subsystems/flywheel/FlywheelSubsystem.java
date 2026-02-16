package com.aembot.frc2026.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.flywheel.FlywheelIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private final FlywheelIO flywheel;

  public FlywheelSubsystem(MotorConfiguration<TalonFXConfiguration> config, FlywheelIO flywheel) {
    super(new MotorInputs(), flywheel.getLeadMotor(), config);
    // super(
    //     new MotorInputs(),
    //     flywheel.getLeadMotor(),

    //     //generateFollowerInputs(flywheel.getFollowerMotors()),
    //     //flywheel.getFollowerMotors(),
    //     config);
    // MotorSubsystem(new MotorInputs(), flywheel.getLeadMotor(), config);

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
    return smartVelocitySetpointCommand(() -> 20).withName("DefaultSpin").ignoringDisable(true);
  }
}
