package com.aembot.frc2026.commands;

import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public final class ShooterCommands {

  private final HoodSubsystem hood;
  private final TurretSubsystem turret;

  public ShooterCommands(HoodSubsystem hood, TurretSubsystem turret) {
    this.hood = hood;
    this.turret = turret;
  }

  public Command createHoodStopCommand() {
    return hood.smartVelocitySetpointCommand(() -> 0);
  }

  public Command createHoodUpCommand() {
    return hood.smartVelocitySetpointCommand(() -> 30);
  }

  public Command createHoodDownCommand() {
    return hood.smartVelocitySetpointCommand(() -> -30);
  }

  public Command createTurretStopCommand() {
    return turret.smartVelocitySetpointCommand(() -> 0);
  }

  public Command createTurretLeftCommand() {
    return turret.smartVelocitySetpointCommand(() -> 30);
  }

  public Command createTurretRightCommand() {
    return turret.smartVelocitySetpointCommand(() -> -30);
  }

  private double getTurretForwardFromRobotPose() {
    double targetRotation =
        RobotStateYearly.get().getLatestFieldRobotPose().getRotation().getDegrees() + 180;

    if (targetRotation < 0) {
      targetRotation += 360;
    }

    return targetRotation;
  }

  public Command createTurretAbsoluteForwardCommand() {
    return turret.smartPositionSetpointCommand(() -> getTurretForwardFromRobotPose());
  }
}
