package com.aembot.lib.state.subsystems.flywheel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.state.RobotState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class SimulatedShooterFlywheelState implements Loggable {
  private final RobotState robotStateInstance;

  /**
   * Runnable to call when we fire a game piece. Should adjust simulated flywheel velocity to
   * account for a gamepiece moving through it.
   */
  private Runnable simulateFireCallback = () -> {};

  private List<Pose3d> loggedTrajectory = new ArrayList<>();

  public SimulatedShooterFlywheelState(RobotState robotStateInstance) {
    this.robotStateInstance = robotStateInstance;
  }

  /**
   * Simulate firing a gamepiece through the shooter.
   *
   * @param gamePieceExitPoint The robot-relative point at which the gamepiece exits the shooter
   */
  public void simulateShot(Transform3d gamePieceExitPoint, double launchSpeedMetersPerSecond) {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                    robotStateInstance.getLatestFieldRobotPose().getTranslation(),
                    gamePieceExitPoint.getTranslation().toTranslation2d(),
                    robotStateInstance.getLatestFusedFieldRelativeChassisSpeed(),
                    gamePieceExitPoint.getRotation().toRotation2d(),
                    Distance.ofBaseUnits(gamePieceExitPoint.getZ(), Meters),
                    LinearVelocity.ofBaseUnits(launchSpeedMetersPerSecond, MetersPerSecond),
                    Angle.ofBaseUnits(
                        gamePieceExitPoint
                            .getRotation()
                            .minus(new Rotation3d(gamePieceExitPoint.getRotation().toRotation2d()))
                            .getY(),
                        Radians))
                .withProjectileTrajectoryDisplayCallBack(
                    (trajectory) -> loggedTrajectory = trajectory));

    simulateFireCallback.run();
  }

  /**
   * Set the callback to be called when a game piece is fired. Should adjust simulated flywheel
   * velocity to account for a gamepiece moving through it.
   */
  public void setSimulateFireCallback(Runnable callback) {
    this.simulateFireCallback = callback;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(
        standardPrefix + "/SimulatedShotTrajectory", loggedTrajectory.toArray(new Pose3d[0]));
  }
}
