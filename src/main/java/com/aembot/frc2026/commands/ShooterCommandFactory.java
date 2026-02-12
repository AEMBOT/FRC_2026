package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.util.OptimalVelocityTable;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import java.util.function.Supplier;

public final class ShooterCommandFactory {

  private final HoodSubsystem hood;

  private final OptimalVelocityTable shootingHubTable;

  public ShooterCommandFactory(HoodSubsystem hood) {
    this.hood = hood;

    String robotType;
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        robotType = "/sim";
        break;

      case REPLAY:

      case REAL:

      default:
        robotType = "/real";
        break;
    }
    ;

    this.shootingHubTable =
        new OptimalVelocityTable(
            Filesystem.getDeployDirectory()
                + "/initial-velocities"
                + robotType
                + "/Shooting_Hub_Initial_Velocities.csv");
  }

  public Command createHoodTowardsHubCommand() {
    Supplier<Rotation3d> rotation =
        () ->
            shootingHubTable.getFuelInitVelocityRotation3d(
                RobotStateYearly.get().getLatestFieldRobotPose(),
                RobotStateYearly.get().getLatestDesiredFieldRelativeChassisSpeed());
    return new RepeatCommand(
        hood.smartPositionSetpointCommand(() -> Units.radiansToDegrees(rotation.get().getY())));
  }
}
