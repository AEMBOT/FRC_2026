package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import com.aembot.lib.tracing.Traced;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilVisionSubsystem extends AEMSubsystem {
  private static final double CAMERA_POSE_LOG_PERIOD_SECONDS = 0.1;
  private static final double VISION_ESTIMATE_LOG_PERIOD_SECONDS = 0.1;
  private static final double VISION_INPUT_LOG_PERIOD_SECONDS = 0.1;

  private static class CameraLogContext {
    public final AprilCameraIO io;
    public final AprilVisionInputs inputs;
    public final boolean isTurretCamera;
    public final String cameraName;
    public final String cameraPositionLogKey;
    public final String processInputsLogKey;

    private CameraLogContext(
        AprilCameraIO io,
        AprilVisionInputs inputs,
        boolean isTurretCamera,
        String cameraName,
        String cameraPositionLogKey,
        String processInputsLogKey) {
      this.io = io;
      this.inputs = inputs;
      this.isTurretCamera = isTurretCamera;
      this.cameraName = cameraName;
      this.cameraPositionLogKey = cameraPositionLogKey;
      this.processInputsLogKey = processInputsLogKey;
    }
  }

  protected final RobotState robotStateInstance;
  protected final List<CameraLogContext> cameraContexts = new ArrayList<>();
  private final List<AprilCameraOutput> aprilTagObservationsBuffer = new ArrayList<>();
  private final String visionObservationCountLogKey;
  private final String latestVisionEstimatedRobotPoseLogKey;
  private final String latencyPeriodicLogKey;
  private final String visionActiveLogKey;

  private boolean visionActive = true;
  private double nextCameraPoseLogTimestampSeconds = 0.0;
  private double nextVisionEstimateLogTimestampSeconds = 0.0;
  private double nextVisionInputLogTimestampSeconds = 0.0;

  public AprilVisionSubsystem(RobotState robotStateInstance, AprilCameraIO... cameras) {
    super("VisionSubsystem");

    this.robotStateInstance = robotStateInstance;
    this.visionObservationCountLogKey = logPrefixStandard + "/VisionObservationCount";
    this.latestVisionEstimatedRobotPoseLogKey =
        logPrefixStandard + "/LatestVisionEstimatedRobotPose";
    this.latencyPeriodicLogKey = logPrefixStandard + "/LatencyPeriodicMS";
    this.visionActiveLogKey = logPrefixStandard + "/VisionActive";

    for (AprilCameraIO camera : cameras) {
      AprilVisionInputs inputs = new AprilVisionInputs();
      String cameraName = camera.getConfiguration().cameraName;
      cameraContexts.add(
          new CameraLogContext(
              camera,
              inputs,
              "turret".equals(cameraName),
              cameraName,
              logPrefixStandard + "/" + cameraName + "/CameraPosition",
              logPrefixInput + "/" + camera.getConfiguration().toString()));
    }

    updateNTDisabled();
  }

  @Override
  @Traced
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    boolean shouldLogCameraPose = timestamp >= nextCameraPoseLogTimestampSeconds;
    if (shouldLogCameraPose) {
      nextCameraPoseLogTimestampSeconds = timestamp + CAMERA_POSE_LOG_PERIOD_SECONDS;
    }
    aprilTagObservationsBuffer.clear();

    for (CameraLogContext context : cameraContexts) {
      if (context.isTurretCamera) continue;

      if (shouldLogCameraPose) {
        Logger.recordOutput(
            context.cameraPositionLogKey,
            new Pose3d(robotStateInstance.getLatestFieldRobotPose())
                // Convert the camera pose into a field-space pose for visualization
                .plus(context.io.getConfiguration().getCameraPosition().minus(Pose3d.kZero)));
      }

      context.io.updateInputs(context.inputs);

      if (context.inputs.coprocessorEstimationLatencyCompensated != null && visionActive) {
        aprilTagObservationsBuffer.add(
            new AprilCameraOutput(
                context.cameraName,
                context.inputs.tagID,
                new VisionPoseEstimation(
                    context.inputs.coprocessorEstimationLatencyUncompensated,
                    context.inputs.coprocessorEstimationLatencyCompensated,
                    context.inputs.coprocessorEstimationStdDevs,
                    context.inputs.coprocessorEstimationTimestamp)));
      }
    }

    robotStateInstance.setApriltagObservations(aprilTagObservationsBuffer);

    updateLog();
    List<AprilCameraOutput> observations = robotStateInstance.getAprilTagObservations();
    Logger.recordOutput(visionObservationCountLogKey, observations.size());
    if (!observations.isEmpty() && timestamp >= nextVisionEstimateLogTimestampSeconds) {
      nextVisionEstimateLogTimestampSeconds = timestamp + VISION_ESTIMATE_LOG_PERIOD_SECONDS;
      Logger.recordOutput(
          latestVisionEstimatedRobotPoseLogKey,
          observations.get(observations.size() - 1).estimatedPose());
    }

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(latencyPeriodicLogKey, (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  public Command createKillVisionCommand() {
    return new InstantCommand(() -> visionActive = false);
  }

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(visionActiveLogKey, visionActive);
    double timestampSeconds = Timer.getFPGATimestamp();
    if (timestampSeconds < nextVisionInputLogTimestampSeconds) {
      return;
    }
    nextVisionInputLogTimestampSeconds = timestampSeconds + VISION_INPUT_LOG_PERIOD_SECONDS;

    for (CameraLogContext context : cameraContexts) {
      Logger.processInputs(context.processInputsLogKey, context.inputs);
    }
  }

  private void updateNTDisabled() {
    for (CameraLogContext context : cameraContexts) {
      context.io.updateNetworkTablesForDisabled();
    }
  }

  private void updateNTEnabled() {
    for (CameraLogContext context : cameraContexts) {
      context.io.updateNetworkTablesForEnabled();
    }
  }

  /**
   * DO NOT CALL PERIODICALLY
   *
   * @return A command that update network tables with values to use while disabled
   */
  public Command updateNTDisabledCommand() {
    return new InstantCommand(this::updateNTDisabled);
  }

  /**
   * DO NOT CALL PERIODICALLY
   *
   * @return A command that update network tables with values to use while enabled
   */
  public Command updateNTEnabledCommand() {
    return new InstantCommand(this::updateNTEnabled);
  }
}
