package com.aembot.frc2026.subsystems;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.io.DrivetrainHardwareIO;
import com.aembot.lib.subsystems.drive.io.DrivetrainIOReplay;
import com.aembot.lib.subsystems.drive.io.DrivetrainSimIO;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIOReplay;
import com.aembot.lib.subsystems.hood.io.HoodSimIO;
import com.aembot.lib.subsystems.hood.io.TalonFXHoodHardwareIO;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.OverBumperIntakeDeployReplayIO;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.OverBumperIntakeDeploySimIO;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.TalonFXOverBumperIntakeDeployHardwareIO;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.io.OverBumperIntakeRollerReplayIO;
import com.aembot.lib.subsystems.intake.over_bumper.run.io.OverBumperIntakeRollerSimIO;
import com.aembot.lib.subsystems.intake.over_bumper.run.io.TalonFXOverBumperIntakeRollerHardwareIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Subsystem factories intended to be called from RobotContainer */
public class SubsystemFactory {
  public static DriveSubsystem createDriveSubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new DriveSubsystem(
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                new DrivetrainSimIO(
                    RobotRuntimeConstants.ROBOT_CONFIG.getSimulatedDrivetrainConfiguration(),
                    RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                    RobotRuntimeConstants.ROBOT_CONFIG.getSwerveConfigurations()),
                RobotStateYearly.get())
            .withSetPose(new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));
      case REPLAY:
        return new DriveSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
            new DrivetrainIOReplay(),
            RobotStateYearly.get());
      case REAL:
      default:
        return new DriveSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
            new DrivetrainHardwareIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                RobotRuntimeConstants.ROBOT_CONFIG.getSwerveConfigurations()),
            RobotStateYearly.get());
    }
  }

  public static HoodSubsystem createHoodSubsystem() {

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new HoodSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getSimHoodConfig(),
            new HoodSimIO(RobotRuntimeConstants.ROBOT_CONFIG.getSimHoodConfig()));

      case REPLAY:
        return new HoodSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig(), new HoodIOReplay());
      case REAL:

      default:
        return new HoodSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig(),
            new TalonFXHoodHardwareIO(RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig()));
    }
  }

  public static OverBumperIntakeDeploySubsystem createIntakeDeploySubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new OverBumperIntakeDeploySubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getSimIntakeDeployConfig().kRealConfig,
            new OverBumperIntakeDeploySimIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getSimIntakeDeployConfig()),
            (state) -> SimulatedRobotStateYearly.get().updateIntakeDeployState(state));
      case REPLAY:
        return new OverBumperIntakeDeploySubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig(),
            new OverBumperIntakeDeployReplayIO(),
            (state) -> RobotStateYearly.get().updateIntakeDeployState(state));
      case REAL:

      default:
        return new OverBumperIntakeDeploySubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig(),
            new TalonFXOverBumperIntakeDeployHardwareIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig()),
            (state) -> RobotStateYearly.get().updateIntakeDeployState(state));
    }
  }

  public static OverBumperIntakeRollerSubsystem createIntakeRollerSubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new OverBumperIntakeRollerSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getSimIntakeRollerConfig().kRealConfig,
            new OverBumperIntakeRollerSimIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getSimIntakeRollerConfig()),
            (state) -> SimulatedRobotStateYearly.get().updateIntakeRollerState(state));
      case REPLAY:
        return new OverBumperIntakeRollerSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig(),
            new OverBumperIntakeRollerReplayIO(),
            (state) -> RobotStateYearly.get().updateIntakeRollerState(state));
      case REAL:

      default:
        return new OverBumperIntakeRollerSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig(),
            new TalonFXOverBumperIntakeRollerHardwareIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig()),
            (state) -> RobotStateYearly.get().updateIntakeRollerState(state));
    }
  }
}
