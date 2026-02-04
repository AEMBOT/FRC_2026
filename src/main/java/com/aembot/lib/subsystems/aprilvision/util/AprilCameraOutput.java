package com.aembot.lib.subsystems.aprilvision.util;

import com.aembot.lib.config.odometry.OdometryStandardDevs;

/**
 * Represents data output of an april-tag tracking camera for pose estimation. Fields:
 *
 * <ul>
 *   <li>{@code cameraName()}: {@link String} Name of the camera that observed the tag. Should be
 *       {@link com.aembot.lib.config.subsystems.vision.CameraConfiguration#cameraName
 *       CameraConfiguration#cameraName}.
 *   <li>{@code tagID()}: {@code int} ID of the observed tag
 *   <li>{@code estimatedPose()}: {@link VisionPoseEstimation} The robot pose estimated from the
 *       observation
 *   <li>{@code stddevs()} {@link OdometryStandardDevs} The standard deviations of the pose
 *       estimation
 */
public record AprilCameraOutput(String cameraName, int tagID, VisionPoseEstimation estimatedPose) {}
