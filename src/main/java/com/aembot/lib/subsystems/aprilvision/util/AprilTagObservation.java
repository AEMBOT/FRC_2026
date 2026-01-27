package com.aembot.lib.subsystems.aprilvision.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents an observation of an april tag. Fields:
 *
 * <ul>
 *   <li>{@code cameraName()}: Name of the camera that observed the tag. Should be {@link
 *       com.aembot.lib.config.subsystems.vision.CameraConfiguration#cameraName
 *       CameraConfiguration#cameraName}.
 *   <li>{@code tagID()}: ID of the observed tag
 *   <li>{@code estimatedPose()}: The robot pose estimated from the observation
 */
public record AprilTagObservation(String cameraName, int tagID, Pose2d estimatedPose) {}
