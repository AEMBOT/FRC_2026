package com.aembot.lib.state;

import com.aembot.lib.constants.RobotStateConstants;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.math.ConcurrentTimeInterpolatableBuffer;
import com.aembot.lib.subsystems.aprilvision.util.AprilTagObservation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/**
 * Robot state class that retains all information that is used to determine the robots current
 * state. This could include robot pose, velocities, mechanism positions, amount of fuel in hopper,
 * etc. For the year-specific implementation of this class, see/create RobotStateYearly in
 * frcXXXX.state.RobotStateYearly.
 *
 * <p>The states stored in this SHOULD ALL BE THREAD SAFE
 */
public abstract class RobotState implements Loggable {
  /** Tracks only values as the relate to robot odometry */
  class Odometry {
    /** Thread safe buffer to track robot pose over time. this pose includes updates from vision */
    public final ConcurrentTimeInterpolatableBuffer<Pose2d> timeInterpolatableEstimatedRobotPose =
        ConcurrentTimeInterpolatableBuffer.createBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);

    /// -- Angular Velocity Time Buffers
    public final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);
    public final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);
    public final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);

    /// -- Robot additional rotation time buffers
    public final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);
    public final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);

    /// -- Robot Acceleration time buffers
    public final ConcurrentTimeInterpolatableBuffer<Double> driveAccelX =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);
    public final ConcurrentTimeInterpolatableBuffer<Double> driveAccelY =
        ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);

    /** Current robot-relative chassis speeds (measured from encoders) */
    public final AtomicReference<ChassisSpeeds> actualRobotRelativeChassisSpeeds =
        new AtomicReference<>(new ChassisSpeeds());

    /** Current field-relative chassis speeds (measured from encoders) */
    public final AtomicReference<ChassisSpeeds> actualFieldRelativeChassisSpeeds =
        new AtomicReference<>(new ChassisSpeeds());

    /** Desired robot-relative chassis speeds (set by control systems) */
    public final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
        new AtomicReference<>(new ChassisSpeeds());

    /** Desired field-relative chassis speeds (set by control systems) */
    public final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
        new AtomicReference<>(new ChassisSpeeds());

    /**
     * Field-centric chassis speeds with gyro rotation rate instead of module-computed (measured
     * from encoders + gyro)
     */
    public final AtomicReference<ChassisSpeeds> gyroFusedChassisSpeeds =
        new AtomicReference<>(new ChassisSpeeds());

    public Odometry() {
      // Add a sample at 0.0 so the bit exists
      this.timeInterpolatableEstimatedRobotPose.addSample(0.0, Pose2d.kZero);
    }

    /**
     * Logs the latest pose in the given time interpolated pose
     *
     * @param key Where to log the result to
     * @param buffer Buffer we are logging the data from
     */
    public static void logTimeInterpolatedPose(
        String key, ConcurrentTimeInterpolatableBuffer<Pose2d> buffer) {
      Entry<Double, Pose2d> latest = buffer.getLatest();
      if (latest != null) {
        Logger.recordOutput(key, latest.getValue());
      }
    }
  }

  /* Normal class properties */
  protected final RobotState.Odometry odometryState = new RobotState.Odometry();

  private final List<AprilTagObservation> aprilTagObservations = new ArrayList<>();

  public void addOdometryMeasurement(double timestamp, Pose2d pose) {
    odometryState.timeInterpolatableEstimatedRobotPose.addSample(timestamp, pose);
  }

  /**
   * Set the list of april tag observations
   *
   * @param observations a list consisting of all the observations for this periodic loop
   */
  public void setApriltagObservations(List<AprilTagObservation> observations) {
    aprilTagObservations.clear();

    for (AprilTagObservation observation : observations) {
      aprilTagObservations.add(observation);
    }
  }

  /**
   * Get the current valid april tag observations
   *
   * @return The list of valid april tag observations
   */
  public List<AprilTagObservation> getAprilTagObservations() {
    return aprilTagObservations;
  }

  /**
   * Retrieve the latest robot field pose from the robot
   *
   * @return Robot field pose
   */
  public Pose2d getLatestFieldRobotPose() {
    var entry = odometryState.timeInterpolatableEstimatedRobotPose.getInternalBuffer().lastEntry();
    if (entry == null) {
      return null;
    }
    return entry.getValue();
  }

  /**
   * Add the current motion measurements for the robot to the time buffer for logging
   *
   * @param timestamp The timestamp for which these measurements were logged
   * @param angularRollRadPerS The angular roll velocity in rads per second
   * @param angularPitchRadsPerS The angular pitch velocity in rads per second
   * @param angularYawRadPerS The angular yaw velocity in rads per second
   * @param pitchRads The current pitch in rads
   * @param rollRads The current roll in rads
   * @param accelX The x acceleration of the robot in meters per second per second
   * @param accelY The Y acceleration of the robot in meters per second per second
   * @param actualRobotRelativeChassisSpeeds The actual measurement of robot speeds from the swerve
   *     modules
   * @param actualFieldRelativeChassisSpeeds The actual measurements of the robot speed converted to
   *     field relative
   * @param desiredRobotRelativeChassisSpeeds The desired robot speeds for the swerve modules
   * @param desiredFieldRelativeChassisSpeeds The desired robot speeds for the swerve modules
   *     relative to the field
   * @param gyroFusedChassisSpeeds The actual measurement of robot speeds relative to the field
   *     using gyro rotation rates instead of modules
   */
  public void addChassisMotionMeasurements(
      double timestamp,
      double angularRollRadPerS,
      double angularPitchRadsPerS,
      double angularYawRadPerS,
      double pitchRads,
      double rollRads,
      double accelX,
      double accelY,
      ChassisSpeeds actualRobotRelativeChassisSpeeds,
      ChassisSpeeds actualFieldRelativeChassisSpeeds,
      ChassisSpeeds desiredRobotRelativeChassisSpeeds,
      ChassisSpeeds desiredFieldRelativeChassisSpeeds,
      ChassisSpeeds gyroFusedChassisSpeeds) {
    // Add entries to time buffers for robot state
    odometryState.driveRollAngularVelocity.addSample(timestamp, angularRollRadPerS);
    odometryState.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
    odometryState.driveYawAngularVelocity.addSample(timestamp, angularYawRadPerS);

    odometryState.drivePitchRads.addSample(timestamp, pitchRads);
    odometryState.driveRollRads.addSample(timestamp, rollRads);

    odometryState.driveAccelX.addSample(timestamp, accelX);
    odometryState.driveAccelY.addSample(timestamp, accelY);

    // Update robot chassis speeds
    // --- Actual speeds
    odometryState.actualRobotRelativeChassisSpeeds.set(actualRobotRelativeChassisSpeeds);
    odometryState.actualFieldRelativeChassisSpeeds.set(actualFieldRelativeChassisSpeeds);

    // --- Actual speeds + Gyro Speeds
    odometryState.gyroFusedChassisSpeeds.set(gyroFusedChassisSpeeds);

    // --- Desired speeds
    odometryState.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
    odometryState.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeChassisSpeeds);
  }

  public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
    return odometryState.actualFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
    return odometryState.actualRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
    return odometryState.actualRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
    return odometryState.actualFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
    return odometryState.gyroFusedChassisSpeeds.get();
  }

  // --- Loggable Implementation ---
  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    RobotState.Odometry.logTimeInterpolatedPose(
        "SensorRobotState/RobotPose2d", odometryState.timeInterpolatableEstimatedRobotPose);
  }
}
