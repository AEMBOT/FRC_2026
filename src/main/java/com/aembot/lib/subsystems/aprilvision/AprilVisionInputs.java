package com.aembot.lib.subsystems.aprilvision;

import com.aembot.lib.math.PositionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AprilVisionInputs implements LoggableInputs {
  public boolean hasTag = false;

  /**
   * Ms between the end of the exposure of the middle row of the sensor to the data being published
   * to network tables
   */
  public double latency;

  // ===== RAW COPROCESSOR DATA (for replay) =====

  /**
   * Raw pose estimate from the vision coprocessor (e.g., Limelight MegaTag2). This is the
   * unprocessed pose before any RIO-side transformations.
   */
  public Pose2d rawCoprocessorPose = PositionUtil.NaN.POSE2D;

  /** Raw standard deviations array from the coprocessor (12-element array for Limelight) */
  public double[] rawStdDevsArray = new double[0];

  /** Average distance to detected tags in meters */
  public double avgTagDist;

  /** Average tag area as percentage of image (0-100) */
  public double avgTagArea;

  /** Number of tags detected */
  public int tagCount;

  /** Timestamp of the coprocessor estimate in seconds */
  public double coprocessorEstimationTimestamp;

  // ===== FRAME TIMING (for measuring true end-to-end vision latency) =====

  /**
   * Raw NetworkTables timestamp (microseconds) of the botpose frame this estimate came from, before
   * any latency subtraction. Logged so the timestamp math can be audited after the fact.
   */
  public long rawNtTimestampMicros;

  /**
   * FPGA time (seconds) at which the botpose frame was received by the RIO. Together with {@link
   * #coprocessorEstimationTimestamp} this gives the true age of an estimate, which cannot be
   * recovered from loop-reference timestamps alone.
   */
  public double frameReceivedFpgaTimestamp;

  // ===== PER-TAG DATA =====

  /**
   * IDs of the tags that contributed to this estimate. Parallel to {@link #tagDistancesToCamera},
   * {@link #tagAmbiguities} and {@link #tagAreas}.
   *
   * <p>Without this, a bad pose solution cannot be attributed to the tags that produced it, which
   * is what distinguishes a bad field map from bad camera extrinsics.
   */
  public int[] tagIDs = new int[0];

  /** Per-tag distance from the camera in meters. Parallel to {@link #tagIDs}. */
  public double[] tagDistancesToCamera = new double[0];

  /** Per-tag pose ambiguity. Parallel to {@link #tagIDs}. */
  public double[] tagAmbiguities = new double[0];

  /** Per-tag area as a percentage of the image. Parallel to {@link #tagIDs}. */
  public double[] tagAreas = new double[0];

  /**
   * False when the botpose array length did not match the expected {@code 11 + 7 * tagCount}, so
   * the per-tag arrays above could not be trusted and were left empty. The header values (pose, tag
   * count, distance, area) are still used. Logged because the array parser otherwise degrades
   * silently to zeroes on a malformed frame.
   */
  public boolean rawArrayLengthValid = true;

  @Override
  public void toLog(LogTable table) {
    table.put("HasTag", hasTag);
    table.put("RawCoprocessorPose", rawCoprocessorPose);
    table.put("RawStdDevsArray", rawStdDevsArray);
    table.put("AvgTagDist", avgTagDist);
    table.put("AvgTagArea", avgTagArea);
    table.put("TagCount", tagCount);
    table.put("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
    table.put("Latency", latency);
    table.put("RawNtTimestampMicros", rawNtTimestampMicros);
    table.put("FrameReceivedFpgaTimestamp", frameReceivedFpgaTimestamp);
    table.put("TagIDs", tagIDs);
    table.put("TagDistancesToCamera", tagDistancesToCamera);
    table.put("TagAmbiguities", tagAmbiguities);
    table.put("TagAreas", tagAreas);
    table.put("RawArrayLengthValid", rawArrayLengthValid);
  }

  @Override
  public void fromLog(LogTable table) {
    hasTag = table.get("HasTag", hasTag);
    rawCoprocessorPose = table.get("RawCoprocessorPose", rawCoprocessorPose);
    rawStdDevsArray = table.get("RawStdDevsArray", rawStdDevsArray);
    avgTagDist = table.get("AvgTagDist", avgTagDist);
    avgTagArea = table.get("AvgTagArea", avgTagArea);
    tagCount = table.get("TagCount", tagCount);
    coprocessorEstimationTimestamp =
        table.get("CoprocessorEstimationTimestamp", coprocessorEstimationTimestamp);
    latency = table.get("Latency", latency);
    rawNtTimestampMicros = table.get("RawNtTimestampMicros", rawNtTimestampMicros);
    frameReceivedFpgaTimestamp =
        table.get("FrameReceivedFpgaTimestamp", frameReceivedFpgaTimestamp);
    tagIDs = table.get("TagIDs", tagIDs);
    tagDistancesToCamera = table.get("TagDistancesToCamera", tagDistancesToCamera);
    tagAmbiguities = table.get("TagAmbiguities", tagAmbiguities);
    tagAreas = table.get("TagAreas", tagAreas);
    rawArrayLengthValid = table.get("RawArrayLengthValid", rawArrayLengthValid);
  }
}
