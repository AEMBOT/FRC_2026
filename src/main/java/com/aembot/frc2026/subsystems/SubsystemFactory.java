package com.aembot.frc2026.subsystems;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.subsystems.aprilvision.AprilVisionSubsystem;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.io.AprilCameraReplayIO;
import com.aembot.lib.subsystems.aprilvision.io.Limelight4IOHardware;
import com.aembot.lib.subsystems.aprilvision.io.Limelight4IOSim;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.io.DrivetrainHardwareIO;
import com.aembot.lib.subsystems.drive.io.DrivetrainIOReplay;
import com.aembot.lib.subsystems.drive.io.DrivetrainSimIO;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIOReplay;
import com.aembot.lib.subsystems.hood.io.HoodSimIO;
import com.aembot.lib.subsystems.hood.io.TalonFXHoodHardwareIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.function.BiFunction;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

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

  public static AprilVisionSubsystem createAprilVisionSubsystem() {
    List<CameraConfiguration> configs =
        RobotRuntimeConstants.ROBOT_CONFIG.getCameraConfigurations();
    List<SimulatedCameraConfiguration> simConfigs =
        RobotRuntimeConstants.ROBOT_CONFIG.getSimulatedCameraConfigurations();

    BiFunction<PhotonCameraSim, Transform3d, VisionSystemSim> simCameraRegistrar =
        (cameraSim, cameraTransform) ->
            SimulatedRobotStateYearly.get().addCameraToVisionSimulation(cameraSim, cameraTransform);

    YearFieldConstantable fieldConstants = new Field2026();

    AprilCameraIO[] cameraIOs = new AprilCameraIO[configs.size()];

    for (int i = 0; i < cameraIOs.length; i++) {
      switch (RobotRuntimeConstants.MODE) {
        case SIM:
          cameraIOs[i] =
              new Limelight4IOSim(
                  simConfigs.get(i), fieldConstants, RobotStateYearly.get(), simCameraRegistrar);
          break;
        case REPLAY:
          cameraIOs[i] = new AprilCameraReplayIO(configs.get(i));
          break;
        default:
        case REAL:
          cameraIOs[i] =
              new Limelight4IOHardware(configs.get(i), fieldConstants, RobotStateYearly.get());
          break;
      }
    }

    return new AprilVisionSubsystem(RobotStateYearly.get(), cameraIOs);
  }
}
