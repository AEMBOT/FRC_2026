package com.aembot.lib.state;

import com.aembot.lib.constants.RobotStateConstants;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.math.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.simulation.PhotonCameraSim;

/**
 * Robot state class that retains all information that is used to determine the robots current state
 * in simulation. This includes robot pose, velocities and mechanism positions, etc. For the
 * year-specific implementation of this class, see/create SimulatedRobotStateYearly in
 * frcXXXX.state.SimulatedRobotStateYearly.
 *
 * <p><strong>The states stored in this SHOULD ALL BE THREAD SAFE</strong>
 *
 * <p><strong>Yearly implementations should implement a constructor that calls</strong> {@code
 * visionSimulation.addAprilTags(FieldXXXX.getInstance().getFieldLayout());}
 *
 * @see RobotState
 */
public abstract class SimulatedRobotState implements Loggable {
  /**
   * Tracks only values pertaining to the position of the robot on the simulated field. Note that
   * these are not estimated, but pertain to the "real" position of the simulated robot. For pose
   * estimation IO, see {@link RobotState.Odometry}
   */
  class PositionState {
    /** Thread safe buffer to track robot pose over time. These are "real" values, not estimated. */
    public final ConcurrentTimeInterpolatableBuffer<Pose2d> timeInterpolatableSimulatedRobotPose =
        ConcurrentTimeInterpolatableBuffer.createBuffer(
            RobotStateConstants.Kinematics.BUFFER_WINDOW_LENGTH);

    public PositionState() {}

    /**
     * Retrieve the latest robot field pose from the simulated robot
     *
     * @return Simulated robot field pose
     */
    public Pose2d getLatestFieldRobotPose() {
      var entry = timeInterpolatableSimulatedRobotPose.getInternalBuffer().lastEntry();
      if (entry == null) {
        return null;
      }
      return entry.getValue();
    }
  }

  protected final SimulatedRobotState.PositionState positionState = new PositionState();

  /**
   * Update the simulated robot state. This is used to tick any simulated robot state functions that
   * need to run during simulationPeriodic
   */
  public void updateState() {
    // TODO Tick vision
  }

  public void addCameraToVisionSimulation(
      PhotonCameraSim simulatedCamera, Transform3d robotToCameraTransform) {
    // TODO addCameraToVisionSimulation
    throw new UnsupportedOperationException("Not yet implemented");
  }

  /**
   * Update the position of the robot. This should be the actual simulated position of the robot on
   * the field.
   */
  public synchronized void updateSimulatedPosition(Pose2d pose) {
    positionState.timeInterpolatableSimulatedRobotPose.addSample(Timer.getFPGATimestamp(), pose);
  }

  /**
   * Retrieve the latest robot field pose from the simulated robot
   *
   * @return Simulated robot field pose
   */
  public Pose2d getLatestFieldRobotPose() {
    var entry = positionState.timeInterpolatableSimulatedRobotPose.getInternalBuffer().lastEntry();
    if (entry == null) {
      return null;
    }
    return entry.getValue();
  }

  // --- Loggable Implementation ---
  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    RobotState.Odometry.logTimeInterpolatedPose(
        "SimulatedRobotState/RobotPose2d", positionState.timeInterpolatableSimulatedRobotPose);
  }
}
