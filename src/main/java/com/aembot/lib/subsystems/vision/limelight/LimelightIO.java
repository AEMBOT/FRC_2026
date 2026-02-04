package com.aembot.lib.subsystems.vision.limelight;

import com.aembot.lib.subsystems.vision.interfaces.CameraIO;
import com.aembot.lib.subsystems.vision.util.VisionStandardDeviations;
import edu.wpi.first.math.geometry.Pose2d;

public interface LimelightIO extends CameraIO {

  /**
   * Does this limelight currently see any april tags
   *
   * @return true if any tags are seen, else false
   */
  public boolean hasTag();

  /**
   * @return the ID of the tag we are currently tracking
   */
  public int getPrimaryTagID();

  /**
   * @return The pose calculate by the megatag 2 algorithm on the limelight
   */
  public Pose2d getEstimatedPose();

  /**
   * @return The number of tags currently visible
   */
  public int getNumTags();

  /**
   * @return The standard deviations of the estimated position
   */
  public VisionStandardDeviations getStdDevs();

  /**
   * Throttle the camera processor.
   *
   * <p>Use 0 while using camera data.
   *
   * <p>Use 100-200 while not using camera data. (e.g. robot is disabled)
   *
   * @param throttle Amount to throttle the camera to
   */
  public void setThrottle(int throttle);
}
