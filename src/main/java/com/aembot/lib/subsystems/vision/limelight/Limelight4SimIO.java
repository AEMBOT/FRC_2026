package com.aembot.lib.subsystems.vision.limelight;

import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.config.camera.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.subsystems.vision.VisionInputs;
import com.aembot.lib.subsystems.vision.util.VisionStandardDeviations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight4SimIO implements LimelightIO {

  private final SimulatedCameraConfiguration kSimCameraConfiguration;

  private final PhotonCamera camera;
  private final PhotonCameraSim simulatedCamera;

  private final Supplier<Rotation2d> kRobotRotationSupplier;

  private final PhotonPoseEstimator poseEstimator;

  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  private double lastPoseEstimateTimestamp = 0;

  public Limelight4SimIO(
      SimulatedCameraConfiguration config,
      YearFieldConstantable fieldConstants,
      Supplier<Rotation2d> robotRotationSupplier,
      BiConsumer<PhotonCameraSim, Transform3d> registerCameraSimConsumer) {
    this.kSimCameraConfiguration = config;

    this.camera = new PhotonCamera(this.kSimCameraConfiguration.toString());
    this.simulatedCamera =
        new PhotonCameraSim(camera, this.kSimCameraConfiguration.kSimCameraProperties);

    this.poseEstimator =
        new PhotonPoseEstimator(
            fieldConstants.getFieldLayout(),
            PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP,
            getRobotToCameraTransform());

    this.kRobotRotationSupplier = robotRotationSupplier;

    registerCameraSimConsumer.accept(simulatedCamera, getRobotToCameraTransform());
  }

  private Transform3d getRobotToCameraTransform() {
    Pose3d mechanismPose = kSimCameraConfiguration.kCameraConfiguration.MechanismPoseSupplier.get();
    Pose3d cameraPose = kSimCameraConfiguration.kCameraConfiguration.CameraPose;

    Pose3d cameraPoseInverse = cameraPose.rotateBy(new Rotation3d(Rotation2d.k180deg));

    return mechanismPose.minus(cameraPoseInverse);
  }

  private PhotonPipelineResult getLastResult() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() != 0) {
      lastResult = results.get(results.size() - 1);
    }

    return lastResult;
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return kSimCameraConfiguration.kCameraConfiguration;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), kRobotRotationSupplier.get());

    inputs.hasTag = hasTag();
    inputs.primaryTagID = getPrimaryTagID();
    inputs.numTags = getNumTags();
    inputs.estimatedRobotPose = getEstimatedPose();
    inputs.stdDevs = getStdDevs();
    inputs.lastEstimateTimestamp = lastResult.getTimestampSeconds();
  }

  @Override
  public boolean hasTag() {

    return getLastResult().hasTargets();
  }

  @Override
  public int getPrimaryTagID() {
    if (getLastResult().getBestTarget() == null) {
      return -1;
    }
    return getLastResult().getBestTarget().fiducialId;
  }

  @Override
  public Pose2d getEstimatedPose() {
    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(getLastResult());

    if (estimatedPose.isPresent()) {
      lastPoseEstimateTimestamp = estimatedPose.get().timestampSeconds;
      return estimatedPose.get().estimatedPose.toPose2d();
    } else {
      return null;
    }
  }

  @Override
  public int getNumTags() {
    return getLastResult().getTargets().size();
  }

  @Override
  public VisionStandardDeviations getStdDevs() {
    int tagCount = getNumTags();

    if (tagCount == 0) {
      return VisionStandardDeviations.ifSeesNoTags();
    }

    double avgTagArea = 0.0;
    for (PhotonTrackedTarget target : getLastResult().getTargets()) {
      avgTagArea += target.getArea();
    }
    avgTagArea /= tagCount;

    double baseXYStdDev = 0.5;
    double baseZStdDev = 0.8;

    double tagCountFactor = 1.0 / Math.sqrt(tagCount);

    double areaFactor = 1.0 / Math.sqrt(avgTagArea);

    double xStdDev = baseXYStdDev * tagCountFactor * areaFactor;
    double yStdDev = baseXYStdDev * tagCountFactor * areaFactor;
    double zStdDev = baseZStdDev * tagCountFactor * areaFactor;

    return new VisionStandardDeviations(xStdDev, yStdDev, zStdDev, 0, 0, 0);
  }

  public void setThrottle(int throttle) {
    // As far as i can tell, there is no way to set the simply throttle of a simulated camera
    // without
  }
}
