package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.constants.FieldConstants;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
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
    assert cornerPositions.size() == 4;

    // Shallow copy the list so we're not sorting the caller's list
    List<Vector2> cornerPositionsSorted = new ArrayList<>();
    for (Vector2 vector2 : cornerPositions) {
      cornerPositionsSorted.add(vector2);
    }

    // Sort corners from lowest to highest. I'm pretty sure cameras guarantee order but I think this
    // is simpler & more robust
    cornerPositionsSorted.sort(
        (first, second) -> {
          // Return the difference as an int. Calc difference, yoink sign, ceil it, and put sign
          // back. Also convert back to int because Math.copySign returns double
          return (int) Math.copySign(Math.ceil(Math.abs(first.y - second.y)), first.y - second.y);
        });

    double bottomY = (cornerPositions.get(0).y + cornerPositions.get(1).y) / 2;
    double topY = (cornerPositions.get(2).y + cornerPositions.get(3).y) / 2;

    return (topY - bottomY);
  }

  /**
   * Compute the rotational based height of a tag from its height in pixels
   *
   * @param tagHeightPixels The height of the tag in pixels
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
   *     center of the tag
   * @param cameraPitch The pitch of the camera. See {@link com.aembot.lib.config.subsystems.vision.CameraConfiguration#getCameraPitch() CameraConfiguration#getCameraPitch()}
   * @return The horizontal distance to the tag in meters. Returns {@link Double#POSITIVE_INFINITY}
   *     if the angle is effectively zero (parallel to the camera) to prevent division by zero.
   */
  public double computeDistanceToTagMeters(
      Rotation2d tagHeightRotations, double tagVertOffset, Rotation2d cameraPitch) {
    double cameraToTag =
        (1 / tagHeightRotations.div(2).getTan()) * (FieldConstants.APRIL_TAG_HEIGHT_METERS / 2);

    Rotation2d cameraToTagRotationWorldSpace =
        Rotation2d.fromRadians(
            Math.asin((FieldConstants.APRIL_TAG_HEIGHT_METERS / 2) / cameraToTag)
                + Math.copySign(getConfiguration().getCameraPitch().getRadians(), tagVertOffset));

    return new Translation2d(cameraToTag, cameraToTagRotationWorldSpace).getX();
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
    return computeDistanceToTagMeters(
        tagHeightRotations,
        (fieldTagHeight) - getConfiguration().getCameraPosition().getZ(),
        getConfiguration().getCameraPitch());
  }
}
