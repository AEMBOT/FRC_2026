package com.aembot.lib.subsystems.aprilvision.util;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Simple class to store the {@link #latencyUncompensatedPose} & {@link #stdDevs()} of a pose
 * estimation from a camera.
 */
public record VisionPoseEstimation(
    Pose2d latencyUncompensatedPose, OdometryStandardDevs stdDevs, double timestampSeconds) {}
