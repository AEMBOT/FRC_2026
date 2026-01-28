package com.aembot.lib.subsystems.aprilvision.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Simple class to store the {@link #latencyUncompensatedPose} and {@link #latencyCompensatedPose}
 * of a pose estimation from a camera.
 */
public record VisionPoseEstimation(
    Pose2d latencyUncompensatedPose, Pose2d latencyCompensatedPose) {}
