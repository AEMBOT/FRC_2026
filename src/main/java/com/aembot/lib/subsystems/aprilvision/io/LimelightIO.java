package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.constants.FieldConstants;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.dyn4j.geometry.Vector2;

/** Abstract class extending {@link AprilCameraIO} with methods specific to Limelights */
public abstract class LimelightIO implements AprilCameraIO {
  /**
   * Compute the height of the april tag in pixels based on the positions of the tag corners
   * utilizing contours to achieve these positions
   *
   * @param cornerPositions The position of the tag on the screen
   * @return The height of the tag in pixels
   */
  protected static double computeTagHeightInPixels(List<Vector2> cornerPositions) {
    /** The height of the lowest point of the tag */
    double minY = Double.POSITIVE_INFINITY;
    /** The height of the highest point of the tag */
    double maxY = Double.NEGATIVE_INFINITY;

    // Iterate thru the corner positions. minY will be the y value of whichever corner has the
    // lowest y, and maxY whichever has the highest y.
    for (Vector2 pos : cornerPositions) {
      double y = pos.y;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }

    return (maxY - minY);
  }

  /**
   * Compute the rotational based height of a tag from its height in pixels
   *
   * @param tagHeightPixels The height of the tag in pixels
   * @param cameraVerticalFOVDegrees The vertical FOV of the camera used to determine the pixel
   *     height
   * @param verticalResolutionPixels The vertical resolution of the camera used to determine the
   *     pixel height
   * @return Rotation2D representing the height of the tag
   */
  protected Rotation2d computeTagHeightInRotations(double tagHeightPixels) {
    return Rotation2d.fromDegrees(
        tagHeightPixels
            * getConfiguration().cameraFOV.verticalDegrees
            / getConfiguration().cameraResolution.heightPixels);
  }

  /**
   * Computes the horizontal distance to an AprilTag using basic trigonometry based on the vertical
   * angle (pitch) to the target. *
   *
   * <p>The calculation uses the formula: {@code d = h / tan(theta)} *
   *
   * <p><strong>Note:</strong> This implementation uses the absolute field height of the AprilTag as
   * the numerator. This assumes the camera is mounted at ground level (height = 0). If the camera
   * is mounted higher, the numerator should be the difference in height {@code (tagHeight -
   * cameraHeight)}.
   *
   * @param tagHeightRotations The height of the apriltag from the perspective of the camera
   *     expressed as an angle
   * @param tagVertOffset The physical vertical offset between the center of the camera and the
   *     bottom of the tag
   * @return The horizontal distance to the tag in meters. Returns {@link Double#POSITIVE_INFINITY}
   *     if the angle is effectively zero (parallel to the camera) to prevent division by zero.
   */
  public static double computeDistanceToTagMetersStatic(
      Rotation2d tagHeightRotations, double tagVertOffset) {
    double tanTheta = tagHeightRotations.getTan();

    if (Math.abs(tanTheta) < 1e-9) {
      return Double.POSITIVE_INFINITY;
    }

    return Math.abs(tagVertOffset / tanTheta);
  }

  /**
   * Computes the horizontal distance to an AprilTag using basic trigonometry based on the vertical
   * angle (pitch) to the target. *
   *
   * <p>The calculation uses the formula: {@code d = h / tan(theta)} *
   *
   * @param tagHeightRotations The height of the apriltag from the perspective of the camera
   *     expressed as an angle
   * @param fieldTagHeight The vertical distance from the floor of the field to the center of the
   *     apriltag
   * @return The horizontal distance to the tag in meters. Returns {@link Double#POSITIVE_INFINITY}
   *     if the angle is effectively zero (parallel to the camera) to prevent division by zero.
   */
  protected double computeDistanceToTagMeters(
      Rotation2d tagHeightRotations, double fieldTagHeight) {
    return computeDistanceToTagMetersStatic(
        tagHeightRotations,
        (fieldTagHeight - FieldConstants.APRIL_TAG_HEIGHT_METERS)
            - getConfiguration().getCameraPosition().getZ());
  }
}
