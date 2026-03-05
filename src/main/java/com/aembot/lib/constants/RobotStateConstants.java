package com.aembot.lib.constants;

/**
 * Constants used to compute information about the current robot state, such as time based odometry
 * buffer memory length
 */
public final class RobotStateConstants {
  /** All constants related to robot state kinematics should reside within this class */
  public class Kinematics {
    /**
     * History size in seconds of RobotState kinematics {@link ConcurrentTimeInterpolatableBuffer}s
     */
    public static final double BUFFER_WINDOW_LENGTH = 1.0;
  }
}
