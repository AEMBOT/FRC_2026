package com.aembot.frc2026.subsystems.vision;

import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import com.aembot.lib.subsystems.vision.VisionInputs;
import com.aembot.lib.subsystems.vision.limelight.LimelightIO;
import com.aembot.lib.subsystems.vision.util.VisionPoseEstimate;
import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends AEMSubsystem {

  private final List<Pair<LimelightIO, VisionInputs>> limelightsWithInputsList = new ArrayList<>();
  private final List<VisionPoseEstimate> limelightPoseEstimates;

  public VisionSubsystem(LimelightIO... limelights) {
    super("VisionSubsystem");

    for (LimelightIO limelight : limelights) {
      limelightsWithInputsList.add(Pair.of(limelight, new VisionInputs()));
    }

    limelightPoseEstimates = new ArrayList<>(limelightsWithInputsList.size());
  }

  @Override
  public void periodic() {
    limelightPoseEstimates.clear();

    for (Pair<LimelightIO, VisionInputs> limelightWithInput : limelightsWithInputsList) {
      VisionInputs inputs = limelightWithInput.getSecond();

      if (inputs.hasTag) {
        VisionPoseEstimate poseEstimate =
            new VisionPoseEstimate(
                inputs.estimatedRobotPose,
                RobotStateYearly.get().getLastUsedMegatagTimestamp(),
                inputs.numTags,
                inputs.stdDevs);
        limelightPoseEstimates.add(poseEstimate);
      }
    }

    Optional<VisionPoseEstimate> fusedPose = Optional.empty();

    if (!limelightPoseEstimates.isEmpty()) {
      fusedPose =
          VisionPoseEstimate.fused(
              (timestamp) -> RobotStateYearly.get().getFieldRobotPoseAtTime(timestamp),
              limelightPoseEstimates);
    }

    if (fusedPose.isPresent()) {
      RobotStateYearly.get().addMegatagEstimateMeasurement(fusedPose.get());
    }

    updateLog();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(
        standardPrefix + "/EstimatedMegatagPose", RobotStateYearly.get().getLatestPoseEstimate());
  }
}
