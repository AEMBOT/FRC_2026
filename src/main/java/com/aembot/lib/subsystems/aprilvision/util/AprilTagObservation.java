package com.aembot.lib.subsystems.aprilvision.util;

/**
 * Represents an observation of an april tag. Fields:
 *
 * <ul>
 *   <li>{@code cameraName()}: {@link String} Name of the camera that observed the tag. Should be
 *       {@link com.aembot.lib.config.subsystems.vision.CameraConfiguration#cameraName
 *       CameraConfiguration#cameraName}.
 *   <li>{@code tagID()}: {@code int} ID of the observed tag
 *   <li>{@code estimatedPose()}: {@link VisionPoseEstimation} The robot pose estimated from the
 *       observation
 */
public record AprilTagObservation(
    String cameraName, int tagID, VisionPoseEstimation estimatedPose) {}
