package com.aembot.lib.subsystems.vision.util;

import com.aembot.lib.config.camera.CameraConfiguration.Location;
import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagObservation {

  /** Name of the camera that made the observation */
  public final String cameraName;

  /** Location of the camera that made the observation */
  public final Location cameraLocation;

  /** ID of the observed tag */
  public final int tagID;

  /** Robot pose estimated based off of position */
  public final Pose2d estimatedRobotPose;

  /**
   * Create a new april tag observation.
   *
   * @param cameraName Name of the camera that made the observation
   * @param cameraLocation Location of the camera that made the observation
   * @param tagID ID of the observed tag
   * @param estimatedRobotPose Robot pose estimated based off of position
   */
  public AprilTagObservation(
      String cameraName, Location cameraLocation, int tagID, Pose2d estimatedRobotPose) {

    this.cameraName = cameraName;
    this.cameraLocation = cameraLocation;
    this.tagID = tagID;
    this.estimatedRobotPose = estimatedRobotPose;
  }
}
