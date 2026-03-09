package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilVisionSubsystem extends AEMSubsystem {
  protected final RobotState robotStateInstance;

  protected final List<Pair<AprilCameraIO, AprilVisionInputs>> camerasWithInputs =
      new ArrayList<>();

  private boolean visionActive = true;

  public AprilVisionSubsystem(RobotState robotStateInstance, AprilCameraIO... cameras) {
    super("VisionSubsystem");

    this.robotStateInstance = robotStateInstance;

    for (AprilCameraIO camera : cameras) {
      camerasWithInputs.add(Pair.of(camera, new AprilVisionInputs()));
    }

    updateNTDisabled();
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    List<AprilCameraOutput> aprilTagObservations = new ArrayList<>();

    for (Pair<AprilCameraIO, AprilVisionInputs> cameraWithInput : camerasWithInputs) {
      AprilCameraIO io = cameraWithInput.getFirst();
      AprilVisionInputs inputs = cameraWithInput.getSecond();

      if (io.getConfiguration().cameraName == "turret") continue;

      Logger.recordOutput(
          logPrefixStandard + "/" + io.getConfiguration().cameraName + "/CameraPosition",
          new Pose3d(robotStateInstance.getLatestFieldRobotPose())
              // This is so jank. why is there no method to add two poses :sob:
              // Convert the cam pos pose2d to a transform2d with #minus, because
              // that returns a Transform2d for some unfathomable reason
              .plus(io.getConfiguration().getCameraPosition().minus(Pose3d.kZero)));

      io.updateInputs(inputs);

      // Log corrected pose - only X correction needed for left/right cameras
      if (inputs.coprocessorEstimationLatencyCompensated != null) {
        double cameraX = io.getConfiguration().getCameraPosition().getX();
        double cameraY = io.getConfiguration().getCameraPosition().getY();
        double heading = inputs.coprocessorEstimationLatencyCompensated.getRotation().getRadians();

        // Only apply X correction to cameras with Y offset (left/right, not back)
        double correctionRobotX = (Math.abs(cameraY) > 0.01) ? -2.0 * cameraX : 0.0;

        // Convert robot-frame X correction to field-frame
        double correctionFieldX = correctionRobotX * Math.cos(heading);
        double correctionFieldY = correctionRobotX * Math.sin(heading);

        Pose2d correctedPose =
            new Pose2d(
                inputs.coprocessorEstimationLatencyCompensated.getX() + correctionFieldX,
                inputs.coprocessorEstimationLatencyCompensated.getY() + correctionFieldY,
                inputs.coprocessorEstimationLatencyCompensated.getRotation());

        Logger.recordOutput(
            logPrefixStandard + "/" + io.getConfiguration().cameraName + "/CorrectedPoseEstimate",
            correctedPose);
      }

      if (inputs.coprocessorEstimationLatencyCompensated != null && visionActive) {
        aprilTagObservations.add(
            new AprilCameraOutput(
                io.getConfiguration().cameraName,
                inputs.tagID,
                new VisionPoseEstimation(
                    inputs.coprocessorEstimationLatencyUncompensated,
                    inputs.coprocessorEstimationLatencyCompensated,
                    inputs.coprocessorEstimationStdDevs,
                    inputs.coprocessorEstimationTimestamp)));
      }
    }

    robotStateInstance.setApriltagObservations(aprilTagObservations);

    updateLog();
    for (AprilCameraOutput observation : robotStateInstance.getAprilTagObservations()) {
      Logger.recordOutput(
          logPrefixStandard + "/VisionEstimatedRobotPose", observation.estimatedPose());
    }

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  public Command createKillVisionCommand() {
    return new InstantCommand(() -> visionActive = false);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {

    Logger.recordOutput(standardPrefix + "/VisionActive", visionActive);

    for (Pair<AprilCameraIO, AprilVisionInputs> cameraWithInput : camerasWithInputs) {
      AprilCameraIO io = cameraWithInput.getFirst();
      AprilVisionInputs inputs = cameraWithInput.getSecond();

      Logger.processInputs(inputPrefix + "/" + io.getConfiguration().toString(), inputs);
    }
  }

  private void updateNTDisabled() {
    for (Pair<AprilCameraIO, AprilVisionInputs> cameraPair : camerasWithInputs) {
      AprilCameraIO camera = cameraPair.getFirst();
      camera.updateNetworkTablesForDisabled();
    }
  }

  private void updateNTEnabled() {
    for (Pair<AprilCameraIO, AprilVisionInputs> cameraPair : camerasWithInputs) {
      AprilCameraIO camera = cameraPair.getFirst();
      camera.updateNetworkTablesForEnabled();
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
