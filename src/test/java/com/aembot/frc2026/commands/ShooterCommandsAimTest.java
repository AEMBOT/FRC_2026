package com.aembot.frc2026.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.aembot.lib.core.phoenix6.AEMSwerveDriveState;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Checks the turret aim against setpoints the robot logged in akit_26-06-02_23-45-14.wpilog (blue
 * alliance). Each sample is the logged drivetrain Pose and robot-relative Speeds input, and the
 * TurretSubsystem SetSmartPositionSetpoint/Position output from the same loop.
 */
class ShooterCommandsAimTest {
  private static final double TOLERANCE_DEGREES = 0.01;

  private static ShooterCommands shooterCommands;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();

    // The aim math never touches the subsystems
    shooterCommands = new ShooterCommands(null, null, null);
    shooterCommands.refreshAlliance();
  }

  /**
   * Build a drivetrain state the way the odometry thread hands it to the fast aim listener
   *
   * @param pose Logged field pose
   * @param robotRelativeSpeeds Logged robot-relative chassis speeds
   * @return The drivetrain state
   */
  private static AEMSwerveDriveState createState(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    AEMSwerveDriveState state = new AEMSwerveDriveState();
    state.Pose = pose;
    state.Speeds = robotRelativeSpeeds;
    return state;
  }

  @Test
  void fastAimMatchesLoggedSetpoints() {
    // t = 66.059400 s, strafing at ~1 m/s
    assertEquals(
        222.9554748513156,
        shooterCommands.computeTurretTarget(
            createState(
                new Pose2d(
                    2.9481494157982038, 3.886786989652106, new Rotation2d(3.1309112119785865)),
                new ChassisSpeeds(-0.07016903077264144, 0.9725381588537481, -0.01777252836215933))),
        TOLERANCE_DEGREES);

    // t = 66.414188 s, just after the stop
    assertEquals(
        189.73600840566928,
        shooterCommands.computeTurretTarget(
            createState(
                new Pose2d(
                    2.9971694924803876, 3.8505972596573685, new Rotation2d(3.130529509630184)),
                new ChassisSpeeds(
                    -0.0021550824097936712, 0.0281280909086109, -0.023070133783784072))),
        TOLERANCE_DEGREES);

    // t = 75.825938 s
    assertEquals(
        182.61993550793062,
        shooterCommands.computeTurretTarget(
            createState(
                new Pose2d(
                    2.5415746450919614, 3.6217959826950925, new Rotation2d(-3.0810764212163466)),
                new ChassisSpeeds(
                    0.05532822873227147, -0.20216015070870863, -0.08077938930430381))),
        TOLERANCE_DEGREES);
  }
}
