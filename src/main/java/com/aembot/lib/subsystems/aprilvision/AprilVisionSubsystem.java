package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.util.AprilTagObservation;
import com.aembot.lib.subsystems.aprilvision.util.VisionPoseEstimation;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilVisionSubsystem extends AEMSubsystem {
  protected final RobotState robotStateInstance;

  protected final List<Pair<AprilCameraIO, AprilVisionInputs>> camerasWithInputs =
      new ArrayList<>();

  public AprilVisionSubsystem(RobotState robotStateInstance, AprilCameraIO... cameras) {
    super("VisionSubsystem");

    this.robotStateInstance = robotStateInstance;

    for (AprilCameraIO camera : cameras) {
      camerasWithInputs.add(Pair.of(camera, new AprilVisionInputs()));
    }
  }

  @Override
  public void periodic() {
    List<AprilTagObservation> aprilTagObservations = new ArrayList<>();

    for (Pair<AprilCameraIO, AprilVisionInputs> cameraWithInput : camerasWithInputs) {
      AprilCameraIO io = cameraWithInput.getFirst();
      AprilVisionInputs inputs = cameraWithInput.getSecond();

      io.updateInputs(inputs);

      if (inputs.hasTag && inputs.robotPoseEstimationLatencyCompensated != null) {
        aprilTagObservations.add(
            new AprilTagObservation(
                io.getConfiguration().cameraName,
                inputs.tagID,
                new VisionPoseEstimation(
                    inputs.robotPoseEstimationLatencyUncompensated,
                    inputs.robotPoseEstimationLatencyCompensated)));
      }
    }

    robotStateInstance.setApriltagObservations(aprilTagObservations);

    updateLog();
    for (AprilTagObservation observation : robotStateInstance.getAprilTagObservations()) {
      Logger.recordOutput(
          logPrefixStandard + "/VisionEstimatedRobotPose", observation.estimatedPose());
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    for (Pair<AprilCameraIO, AprilVisionInputs> cameraWithInput : camerasWithInputs) {
      AprilCameraIO io = cameraWithInput.getFirst();
      AprilVisionInputs inputs = cameraWithInput.getSecond();

      Logger.processInputs(inputPrefix + "/" + io.getConfiguration().toString(), inputs);
    }
  }
}
