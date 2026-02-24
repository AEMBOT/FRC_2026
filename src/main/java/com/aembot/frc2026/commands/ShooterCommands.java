package com.aembot.frc2026.commands;

import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import java.util.Set;

public final class ShooterCommands {

  private final HoodSubsystem hood;
  private final TurretSubsystem turret;
  private final FlywheelSubsystem flywheel;

  public ShooterCommands(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel) {
    this.hood = hood;
    this.turret = turret;
    this.flywheel = flywheel;
  }

  public Command createHoodHoldPositionCommand() {
    // return hood.smartPositionSetpointCommand(hood::getCurrentPosition);
    return new DeferredCommand(
        () -> {
          double pos = hood.getCurrentPosition();
          return hood.smartPositionSetpointCommand(() -> pos);
        },
        Set.of(hood));
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

  public Command createFlywheelSlowSpinCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> 10);
  }

  public Command createFlywheelFastSpinCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> 20);
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
