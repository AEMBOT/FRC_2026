package com.aembot.frc2026.subsystems;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.Field2025;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.frc2026.subsystems.vision.VisionSubsystem;
import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.config.camera.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.subsystems.vision.limelight.Limelight4IOHardware;
import com.aembot.lib.subsystems.vision.limelight.Limelight4SimIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;

public class SubsystemFactory {

  public static VisionSubsystem createVisionSubsystem() {

    List<CameraConfiguration> cameraConfigs =
        RobotRuntimeConstants.ROBOT_CONFIG.getCameraConfigurations();
    List<SimulatedCameraConfiguration> simConfigs =
        RobotRuntimeConstants.ROBOT_CONFIG.getSimulatedCameraConfigurations();

    YearFieldConstantable fieldConstants = new Field2025();

    BiConsumer<PhotonCameraSim, Transform3d> cameraSimRegistrar =
        (cameraSim, cameraTransform) ->
            SimulatedRobotStateYearly.get().addCameraToVisionSimulation(cameraSim, cameraTransform);

    Supplier<Rotation2d> simulatedRobotRotationSupplier =
        () -> SimulatedRobotStateYearly.get().getLatestFieldRobotPose().getRotation();

    Supplier<Rotation2d> robotRotationSupplier =
        () -> RobotStateYearly.get().getLatestFieldRobotPose().getRotation();

    Supplier<Double> robotAngularVelocitySupplier =
        () ->
            Units.radiansToDegrees(
                RobotStateYearly.get()
                    .getLatestDesiredRobotRelativeChassisSpeeds()
                    .omegaRadiansPerSecond);

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new VisionSubsystem(
            new Limelight4SimIO(
                simConfigs.get(0),
                fieldConstants,
                simulatedRobotRotationSupplier,
                cameraSimRegistrar),
            new Limelight4SimIO(
                simConfigs.get(1),
                fieldConstants,
                simulatedRobotRotationSupplier,
                cameraSimRegistrar),
            new Limelight4SimIO(
                simConfigs.get(2),
                fieldConstants,
                simulatedRobotRotationSupplier,
                cameraSimRegistrar),
            new Limelight4SimIO(
                simConfigs.get(3),
                fieldConstants,
                simulatedRobotRotationSupplier,
                cameraSimRegistrar));

      case REPLAY:

      case REAL:

      default:
        return new VisionSubsystem(
            new Limelight4IOHardware(
                cameraConfigs.get(1),
                fieldConstants,
                robotRotationSupplier,
                robotAngularVelocitySupplier),
            new Limelight4IOHardware(
                cameraConfigs.get(2),
                fieldConstants,
                robotRotationSupplier,
                robotAngularVelocitySupplier),
            new Limelight4IOHardware(
                cameraConfigs.get(3),
                fieldConstants,
                robotRotationSupplier,
                robotAngularVelocitySupplier),
            new Limelight4IOHardware(
                cameraConfigs.get(4),
                fieldConstants,
                robotRotationSupplier,
                robotAngularVelocitySupplier));
    }
  }
}
