package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.math.PositionUtil;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.util.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * IO layer for simulating a limelight. Extends {@link Limelight4IOHardware}, publishing data from
 * PhotonVision's sim to the Limelight's expected network table.
 */
public class Limelight4IOSim extends Limelight4IOHardware {
  /* ---- NETWORK TABLES ENTRIES ---- */
  /** bool NT entry indicating whether we have a tag targetted. "tv" on network table */
  protected final NetworkTableEntry validTagEntry;

  /**
   * double NT entry indicating horizontal offset in degrees from crosshair to target. "tx" on
   * network table
   */
  protected final NetworkTableEntry xOffsetEntry;

  /** int NT entry indicating id of the targeted apriltag. "tid" on network table */
  protected final NetworkTableEntry tagIDEntry;

  /**
   * double[] NT entry indicating corner coordinates in pixels [x0,y0,x1,y1......]. "tcornxy" on
   * network table.
   */
  protected final NetworkTableEntry tagCornerPositionsEntry;

  /**
   * int NT entry indicating number of frames to skip between processed frames to reduce temperature
   * rise. "throttle_set" on network table
   */
  protected final NetworkTableEntry throttleSetEntry;

  /**
   * double NT entry indicating capture pipeline latency (ms). Time between the end of the exposure
   * of the middle row of the sensor to the beginning of the tracking pipeline. Sum with {@link
   * #pipelineLatencyEntry} to get total latency. "cl" on network table.
   */
  protected final NetworkTableEntry captureLatencyEntry;

  /**
   * double NT entry indicating The pipeline's latency contribution (ms). Sum with {@link
   * #captureLatencyEntry} to get total latency. "tl" on network table (for some reason).
   */
  protected final NetworkTableEntry pipelineLatencyEntry;

  protected final NetworkTableEntry robotOrientationEntry;

  protected final NetworkTableEntry megatag2PoseEstimateEntry;

  /**
   * double[] NT entry with all valid (unfiltered) fiducials in format [id, txnc, tync, ta,
   * distToCamera, distToRobot, ambiguity, id2.....]
   */
  protected final NetworkTableEntry rawFiducialsEntry;

  /* ---- END NETWORK TABLES ENTRIES ---- */

  private final VisionSystemSim visionSystemSim;

  private final SimulatedCameraConfiguration simConfig;

  private final PhotonCamera photonCamera;
  private final PhotonCameraSim photonCameraSim;

  private final PhotonPoseEstimator photonPoseEstimator;

  /** Heartbeat value of the simulated limelight. Resets at 2 billion */
  private int heartbeat = 0;

  /**
   * Constructor go brrr
   *
   * @param config Config for the simulated camera
   * @param fieldConstants This season's field constants
   * @param robotStateInstance This year's instance of {@link RobotState}
   * @param visionSimulationRegistrar A function for registering the simulated photonvision camera
   *     with the vision simulation system, returning said vision simulation system
   */
  public Limelight4IOSim(
      SimulatedCameraConfiguration config,
      YearFieldConstantable fieldConstants,
      RobotState robotStateInstance,
      BiFunction<PhotonCameraSim, Transform3d, VisionSystemSim> visionSimulationRegistrar) {
    super(config.cameraConfiguration, fieldConstants, robotStateInstance);
    this.simConfig = config;

    this.photonCamera = new PhotonCamera(simConfig.cameraConfiguration.cameraName);
    this.photonCameraSim = new PhotonCameraSim(photonCamera, simConfig.simCameraProperties);

    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            fieldConstants.getFieldLayout(),
            PoseStrategy.CONSTRAINED_SOLVEPNP,
            PositionUtil.toTransform3d(config.cameraConfiguration.getCameraPosition()));

    this.visionSystemSim =
        visionSimulationRegistrar.apply(
            photonCameraSim, PositionUtil.toTransform3d(cameraConfiguration.getCameraPosition()));

    validTagEntry = networkTable.getEntry("tv");
    xOffsetEntry = networkTable.getEntry("tx");
    tagIDEntry = networkTable.getEntry("tid");
    tagCornerPositionsEntry = networkTable.getEntry("tcornxy");
    throttleSetEntry = networkTable.getEntry("throttle_set");
    captureLatencyEntry = networkTable.getEntry("cl");
    pipelineLatencyEntry = networkTable.getEntry("tl");
    robotOrientationEntry = networkTable.getEntry("robot_orientation_set");
    megatag2PoseEstimateEntry = networkTable.getEntry("botpose_orb_wpiblue");
    rawFiducialsEntry = networkTable.getEntry("rawfiducials");
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    this.visionSystemSim.adjustCamera(
        photonCameraSim, PositionUtil.toTransform3d(this.cameraConfiguration.getCameraPosition()));
    this.photonPoseEstimator.setRobotToCameraTransform(
        PositionUtil.toTransform3d(this.cameraConfiguration.getCameraPosition()));

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    if (results.size() > 0) {
      PhotonPipelineResult result = results.get(results.size() - 1);

      // Not sure if this is the best way to do latency
      captureLatencyEntry.setDouble(Timer.getFPGATimestamp() - result.getTimestampSeconds());

      PhotonTrackedTarget target = result.hasTargets() ? result.getBestTarget() : null;

      validTagEntry.setInteger(target != null ? 1 : 0);
      if (target != null) {
        int tagID = target.getFiducialId();
        tagIDEntry.setInteger(tagID);

        double xOffset = target.getYaw();
        xOffsetEntry.setDouble(xOffset);

        List<TargetCorner> targetCorners = target.getDetectedCorners();

        double[] publishedArray = new double[8];
        if (targetCorners.size() == 4) {
          for (int i = 0; i < 4; i++) {
            publishedArray[i * 2] = targetCorners.get(i).x;
            publishedArray[i * 2 + 1] = targetCorners.get(i).y;
          }
        }

        tagCornerPositionsEntry.setDoubleArray(publishedArray);
      }

      updateFiducials(result);

      /* --- "Coprocessor" pose estimation --- */
      updateMegatag2(result);

      heartbeat++;
      if (heartbeat > 2e9) heartbeat = 0;
      heartbeatEntry.setDouble(heartbeat);
    }

    super.updateInputs(inputs);
  }

  protected void updateMegatag2(PhotonPipelineResult result) {
    /* --- "Coprocessor" pose estimation --- */
    double[] orientationArr = robotOrientationEntry.getDoubleArray((double[]) null);

    if (orientationArr != null) {
      photonPoseEstimator.addHeadingData(
          Timer.getFPGATimestamp(), Rotation2d.fromDegrees(orientationArr[0]));

      Optional<EstimatedRobotPose> estimatedPoseOptional = photonPoseEstimator.update(result);

      if (estimatedPoseOptional.isPresent()) {
        EstimatedRobotPose data = estimatedPoseOptional.get();

        Pose3d estimatedPose = data.estimatedPose;

        double avgDistance = 0;
        // Avg area of tags as % of img
        double avgTagArea = 0;
        for (PhotonTrackedTarget targetUsed : data.targetsUsed) {
          avgDistance += targetUsed.bestCameraToTarget.getTranslation().getNorm();
          avgTagArea += targetUsed.area / 100; // 0-100 -> 0-1
        }

        avgDistance /= data.targetsUsed.size();
        avgTagArea /= data.targetsUsed.size();

        /*
         * Array to be published to NT4. In format: Translation (X,Y,Z) in meters
         * Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average
         * tag distance from camera, average tag area (percentage of image)
         */
        megatag2PoseEstimateEntry.setDoubleArray(
            new Double[] {
              estimatedPose.getX(),
              estimatedPose.getY(),
              estimatedPose.getZ(),
              Units.radiansToDegrees(estimatedPose.getRotation().getX()),
              Units.radiansToDegrees(estimatedPose.getRotation().getY()),
              Units.radiansToDegrees(estimatedPose.getRotation().getZ()),
              captureLatencyEntry.getDouble(0),
              (double) data.targetsUsed.size(),
              0.0, // I'm not entirely sure what span is or how to get it, so... hopefully it's
              // not
              // important?
              avgDistance,
              avgTagArea
            });
      } else {
        // LimelightHelpers will correctly read this as no data
        megatag2PoseEstimateEntry.setDoubleArray(new Double[0]);
      }
    }
  }

  public void updateFiducials(PhotonPipelineResult result) {
    List<Double> rawFiducials = new ArrayList<>();

    for (PhotonTrackedTarget target : result.targets) {
      // id, tx deg, ty deg, tag area as percentage (0-1), distToCamera (m), distToRobot (m),
      // ambiguity (0-1)
      rawFiducials.add((double) target.fiducialId);
      rawFiducials.add(-target.yaw); // Inverted between LL & Photon
      rawFiducials.add(target.pitch);
      rawFiducials.add(target.area / 100); // 0-100 -> 0-1
      rawFiducials.add(target.bestCameraToTarget.getTranslation().getNorm());
      rawFiducials.add(
          target
              .bestCameraToTarget
              .inverse()
              .plus(PositionUtil.toTransform3d(cameraConfiguration.getCameraPosition()))
              .getTranslation()
              .getNorm());
      rawFiducials.add(
          target
              .poseAmbiguity); // TODO check this maps correctly to LL ambiguity vals (I suspect it
      // won't TwT)
    }

    rawFiducialsEntry.setDoubleArray(rawFiducials.toArray(new Double[rawFiducials.size() / 7]));
  }

  @Override
  public OdometryStandardDevs getStdDevs(PoseEstimate estimate) {
    // Yoinked from 2481
    double stdDevFactor = Math.pow(estimate.avgTagDist, 2) / estimate.tagCount;

    double translationStddev = cameraConfiguration.baselineTranslationalStdDev * stdDevFactor;
    Double angularStddev = cameraConfiguration.baselineAngularStdDev * stdDevFactor;

    return adjustStdDevsWithOdomPose(
        new OdometryStandardDevs(translationStddev, translationStddev, angularStddev),
        estimate.timestampSeconds,
        estimate.pose);
  }

  @Override
  public void updateNetworkTablesForDisabled() {
    // TODO sim impl
  }

  @Override
  public void updateNetworkTablesForEnabled() {
    // TODO sim impl
  }
}
