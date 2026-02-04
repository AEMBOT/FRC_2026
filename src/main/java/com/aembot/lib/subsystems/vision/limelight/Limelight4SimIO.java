package com.aembot.lib.subsystems.vision.limelight;

import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.config.camera.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.subsystems.vision.VisionInputs;
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

public class Limelight4SimIO implements LimelightIO {

  private final SimulatedCameraConfiguration kSimCameraConfiguration;

  private final PhotonCamera camera;
  private final PhotonCameraSim simulatedCamera;

  private final Supplier<Rotation2d> kRobotRotationSupplier;

  private final PhotonPoseEstimator poseEstimator;

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
    return results.get(results.size() - 1);
  }

  public CameraConfiguration getConfiguration() {
    return kSimCameraConfiguration.kCameraConfiguration;
  }

  public void updateInputs(VisionInputs inputs) {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), kRobotRotationSupplier.get());

    inputs.hasTag = hasTag();
    inputs.primaryTagID = getPrimaryTagID();
    Pose2d estimatedPose = getEstimatedPose();
    if (estimatedPose != null) {
      inputs.estimatedRobotPose = estimatedPose;
    }
  }

  public boolean hasTag() {
    return getLastResult().hasTargets();
  }

  public int getPrimaryTagID() {
    return getLastResult().getBestTarget().fiducialId;
  }

  public Pose2d getEstimatedPose() {
    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(getLastResult());

    if (estimatedPose.isPresent()) {
      return estimatedPose.get().estimatedPose.toPose2d();
    } else {
      return null;
    }
  }

  public void setThrottle(int throttle) {
    // As far as i can tell, there is no way to set the simply throttle of a simulated camera
    // without
  }
}
