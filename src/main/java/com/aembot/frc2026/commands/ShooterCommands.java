package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.frc2026.util.OptimalVelocityTable;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.phoenix6.AEMSwerveDriveState;
import com.aembot.lib.math.PositionUtil;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class ShooterCommands {

  private final HoodSubsystem hood;
  private final TurretSubsystem turret;
  private final FlywheelSubsystem flywheel;

  private final OptimalVelocityTable shootingHubTable;
  private final OptimalVelocityTable passingCornerLeftTable;
  private final OptimalVelocityTable passingCornerRightTable;
  private final OptimalVelocityTable passingCenterLeftTable;
  private final OptimalVelocityTable passingCenterRightTable;

  // Volatile fields are also read by the turret aim on the drivetrain odometry thread
  private volatile Supplier<OptimalVelocityTable> passingTableSupplier;

  private Supplier<Translation2d> shotPositionSupplier;

  // While true, aim as if the robot were at TOWER_SHOT_POSE
  private volatile boolean towerShotActive = false;

  // Alliance for the turret aim, cached once per loop so the odometry thread never queries the
  // DriverStation
  private volatile boolean redAlliance = false;
  private volatile boolean blueAlliance = false;

  // Flywheel boost in m/s as a linear function of distance to target, tunable from SmartDashboard
  private static final String BOOST_SLOPE_KEY = "ShooterBoost/Slope";
  private static final String BOOST_INTERCEPT_KEY = "ShooterBoost/Intercept";
  private double shooterBoostSlope = 0.624737;
  private double shooterBoostIntercept = 2.60562;

  // private double tempShooterBoost = 3.5;

  // private double PASSING_BOOST = 4.5;

  private Translation2d HUB_TRANSLATION = new Translation2d(4.6101, 4.03479);
  private Translation2d PASSING_CORNER_LEFT_POS = new Translation2d(1, 7.069326);
  private Translation2d PASSING_CORNER_RIGHT_POS = new Translation2d(1, 1);
  private Translation2d PASSING_CENTER_LEFT_POS = new Translation2d(2.312797, 5.558536);
  private Translation2d PASSING_CENTER_RIGHT_POS = new Translation2d(2.312797, 2.51079);

  private final Pose2d TOWER_SHOT_POSE = new Pose2d(1.541, 3.709, Rotation2d.kZero);

  // Offset for the turret in case it gets off for whatever reason
  // private volatile double turretOffset = 0.0;

  private boolean turretManual = false;

  /** Manually applied offset to target hood angle for human adjustment ranging from -1.0 to 1.0 */
  private final DoubleSupplier hoodOffsetAxisSupplier;

  private final DoubleSupplier turretManualXAxisSupplier;
  private final DoubleSupplier turretManualYAxisSupplier;

  public ShooterCommands(
      HoodSubsystem hood,
      TurretSubsystem turret,
      FlywheelSubsystem flywheel,
      DoubleSupplier hoodOffsetSupplier,
      DoubleSupplier turretXAxisSupplier,
      DoubleSupplier turretYAxisSupplier) {
    this.hood = hood;
    this.turret = turret;
    this.flywheel = flywheel;
    this.hoodOffsetAxisSupplier = hoodOffsetSupplier;
    this.turretManualXAxisSupplier = turretXAxisSupplier;
    this.turretManualYAxisSupplier = turretYAxisSupplier;

    SmartDashboard.putNumber(BOOST_SLOPE_KEY, shooterBoostSlope);
    SmartDashboard.putNumber(BOOST_INTERCEPT_KEY, shooterBoostIntercept);

    String velocityTableDirectory = Filesystem.getDeployDirectory() + "/initial-velocities/real/";
    if (RobotRuntimeConstants.MODE == RuntimeMode.SIM) {
      velocityTableDirectory = Filesystem.getDeployDirectory() + "/initial-velocities/sim/";
    }

    // Override shooting hub table to always use real trajectories
    this.shootingHubTable =
        new OptimalVelocityTable(
            velocityTableDirectory + "../real/Shooting_Hub_Initial_Velocities.csv");
    this.passingCornerLeftTable =
        new OptimalVelocityTable(
            velocityTableDirectory + "Passing_Left_Corner_Initial_Velocities.csv");
    this.passingCornerRightTable =
        new OptimalVelocityTable(
            velocityTableDirectory + "Passing_Right_Corner_Initial_Velocities.csv");
    this.passingCenterLeftTable =
        new OptimalVelocityTable(
            velocityTableDirectory + "Passing_Left_Center_Initial_Velocities.csv");
    this.passingCenterRightTable =
        new OptimalVelocityTable(
            velocityTableDirectory + "Passing_Right_Center_Initial_Velocities.csv");
    passingTableSupplier = () -> passingCornerRightTable;

    shotPositionSupplier = () -> PASSING_CORNER_RIGHT_POS;
  }

  public void logCommands() {
    AEMLogger.recordOutput(
        "Commands/ShooterCommands/turretOffset", turretManualXAxisSupplier.getAsDouble());

    shooterBoostSlope = SmartDashboard.getNumber(BOOST_SLOPE_KEY, shooterBoostSlope);
    shooterBoostIntercept = SmartDashboard.getNumber(BOOST_INTERCEPT_KEY, shooterBoostIntercept);
  }

  /**
   * Computes the turret's field pose by applying the turret origin offset to the robot pose. This
   * accounts for the turret not being at the robot center.
   *
   * @param robotPose The robot pose to offset
   * @return The turret's pose on the field, or null if robot pose is unavailable
   */
  private Pose2d getTurretFieldPose(Pose2d robotPose) {
    if (robotPose == null) {
      return null;
    }

    Pose3d turretOrigin = RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig().kTurretOriginPose;
    Translation2d turretOffset = new Translation2d(turretOrigin.getX(), turretOrigin.getY());
    Translation2d fieldOffset = turretOffset.rotateBy(robotPose.getRotation());

    return new Pose2d(robotPose.getTranslation().plus(fieldOffset), robotPose.getRotation());
  }

  /**
   * @param robotPose The measured robot pose
   * @return The pose to aim from: the tower shot pose while it is active, otherwise robotPose
   */
  private Pose2d getAimPose(Pose2d robotPose) {
    return towerShotActive ? TOWER_SHOT_POSE : robotPose;
  }

  /**
   * Our shooting zones are different whether we are blue or red
   *
   * @param robotPose The measured robot pose
   * @param blueAlliance True if we are on the blue alliance
   * @return True if the robot is in our shooting zone
   */
  private boolean isInShootingZone(Pose2d robotPose, boolean blueAlliance) {
    return blueAlliance ? robotPose.getX() < 4.02844 : robotPose.getX() > 12.512548;
  }

  /* ---- VELOCITY TABLES ---- */

  /**
   * @param robotPose The measured robot pose
   * @param blueAlliance True if we are on the blue alliance
   * @return The current velocity table to use for aiming
   */
  private OptimalVelocityTable getCurrentVelocityTable(Pose2d robotPose, boolean blueAlliance) {

    // Check if there are no robot pose measurements, mostly applicable at start of program runtime
    if (robotPose == null) {
      return passingTableSupplier.get();
    }

    if (isInShootingZone(robotPose, blueAlliance)) {
      return shootingHubTable;
    } else {
      return passingTableSupplier.get();
    }
  }

  /**
   * @return a command that sets the passing position to the outpost
   */
  public Command createSetPassingPoseCenterRightCommand() {
    return new InstantCommand(
        () -> {
          System.out.println("Setting passing table to center right");
          passingTableSupplier = () -> passingCenterRightTable;
          shotPositionSupplier = () -> PASSING_CENTER_RIGHT_POS;
        });
  }

  /**
   * @return a command that sets the passing position to the left
   */
  public Command createSetPassingPoseCornerLeftCommand() {
    return new InstantCommand(
        () -> {
          System.out.println("Setting passing table to corner right");
          passingTableSupplier = () -> passingCornerLeftTable;
          shotPositionSupplier = () -> PASSING_CORNER_LEFT_POS;
        });
  }

  /**
   * @return a command that sets the passing position to the middle
   */
  public Command createSetPassingPoseCornerRightCommand() {
    return new InstantCommand(
        () -> {
          System.out.println("Setting passing table to corner right");
          passingTableSupplier = () -> passingCornerRightTable;
          shotPositionSupplier = () -> PASSING_CORNER_RIGHT_POS;
        });
  }

  /**
   * @return a command that sets the passing position to the right
   */
  public Command createSetPassingPoseCenterLeftCommand() {
    return new InstantCommand(
        () -> {
          System.out.println("Setting passing table to center left");
          passingTableSupplier = () -> passingCenterLeftTable;
          shotPositionSupplier = () -> PASSING_CENTER_LEFT_POS;
        });
  }

  /**
   * @return the current optimal pitch to shoot to the goal position
   */
  private double getCurrentPitch() {
    Pose2d robotPose = RobotStateYearly.get().getLatestFieldRobotPose();
    double hoodOffsetDegrees = hoodOffsetAxisSupplier.getAsDouble() * 30;
    return Units.radiansToDegrees(
            getCurrentVelocityTable(robotPose, RobotRuntimeConstants.isBlueAlliance())
                .getFuelInitVelocityRotation3d(
                    getTurretFieldPose(getAimPose(robotPose)),
                    RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds())
                .getY())
        + hoodOffsetDegrees;
  }

  /**
   * Function to get an amount to artifically boost flywheel speed
   *
   * <p>NOTE: currently just a static number but in future we may want to scale with distance or
   * something
   *
   * @return amount to boost flywheel speed in m/s
   */
  private double getFlywheelSpeedBoost() {
    // this.tempShooterBoost = SmartDashboard.getNumber("ShooterBoost", tempShooterBoost);
    double boost;
    Pose2d robotPose = RobotStateYearly.get().getLatestFieldRobotPose();
    var pos = robotPose.getTranslation();
    double dist;
    if (isInShootingZone(robotPose, RobotRuntimeConstants.isBlueAlliance())) {
      dist = pos.getDistance(PositionUtil.flipForAlliance(HUB_TRANSLATION));
    } else {
      dist = pos.getDistance(PositionUtil.flipForAlliance(shotPositionSupplier.get()));
    }
    boost = shooterBoostSlope * dist + shooterBoostIntercept;
    AEMLogger.recordOutput("Commands/ShooterCommands/BoostDistance", dist);
    AEMLogger.recordOutput("Commands/ShooterCommands/Boost", boost);

    return (RobotRuntimeConstants.MODE == RuntimeMode.REAL) ? boost : 0.4;
    // return tempShooterBoost;
  }

  /**
   * @return the current optimal speed to shoot to the goal position
   */
  private double getCurrentSpeed() {
    Pose2d robotPose = RobotStateYearly.get().getLatestFieldRobotPose();
    double tableSpeed =
        getCurrentVelocityTable(robotPose, RobotRuntimeConstants.isBlueAlliance())
            .getFuelInitVelocityMagnitude(
                getTurretFieldPose(getAimPose(robotPose)),
                RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());
    AEMLogger.recordOutput("Commands/ShooterCommands/TableSpeed", tableSpeed);
    return tableSpeed + getFlywheelSpeedBoost();
  }

  /* ---- HOOD COMMANDS ---- */

  /**
   * @return a command that sets the hood goal angle to the optimal shooting pitch
   */
  public Command createHoodTowardsGoalCommand() {
    return hood.smartPositionSetpointCommand(() -> getCurrentPitch());
  }

  public Command createHoodDownCommand() {
    return hood.smartPositionSetpointCommand(() -> 90);
  }

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return true if hood is within 10 degrees of target position, false otherwise
   */
  public boolean isHoodNearGoal() {
    double tolerance = RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig().kAutoAimLeniance;
    return MathUtil.isNear(hood.getCurrentPosition(), getCurrentPitch(), tolerance);
  }

  /* ---- TURRET COMMANDS ---- */

  /**
   * Robot-relative angle of the optimal yaw. Reads only its arguments and fields that are safe to
   * read from another thread, so it can run on the drivetrain odometry thread.
   *
   * @param robotPose Field pose of the robot
   * @param fieldSpeeds Field-relative chassis speeds of the robot
   * @return Turret target in degrees, 0 to 360
   */
  public double computeTurretTarget(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    // Read the cached alliance once so the whole calculation agrees on it
    boolean red = redAlliance;

    double targetRotation = 0.0;

    if (!turretManual) {
      targetRotation =
          getCurrentVelocityTable(robotPose, blueAlliance)
                  .getFuelInitVelocityRotation3d(
                      getTurretFieldPose(getAimPose(robotPose)), fieldSpeeds, red)
                  .toRotation2d()
                  .minus(robotPose.getRotation())
                  .getDegrees()
              + (turretManualXAxisSupplier.getAsDouble() * 90);
    } else {
      targetRotation =
          Math.atan2(
                  turretManualYAxisSupplier.getAsDouble(), turretManualXAxisSupplier.getAsDouble())
              - robotPose.getRotation().getRadians();
      targetRotation = -Units.radiansToDegrees(targetRotation);
    }

    // Because of the way the the auto aim tables are set up, need to rotate turret 180 when on red
    // alliance
    if (red) {
      targetRotation += 180;
    }

    targetRotation = MathUtil.inputModulus(targetRotation, 0, 360);

    double scaledValue = ((targetRotation - 180) * 0.1);

    return MathUtil.inputModulus(targetRotation + scaledValue, 0, 360);
  }

  /**
   * {@link #computeTurretTarget(Pose2d, ChassisSpeeds)} from a drivetrain state, for the fast aim
   * listener on the odometry thread
   *
   * @param state Latest drivetrain state, with robot-relative speeds
   * @return Turret target in degrees, 0 to 360
   */
  public double computeTurretTarget(AEMSwerveDriveState state) {
    return computeTurretTarget(
        state.Pose, ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()));
  }

  /** Cache the alliance for the turret aim. Must be called from the main thread. */
  void refreshAlliance() {
    redAlliance = RobotRuntimeConstants.isRedAlliance();
    blueAlliance = RobotRuntimeConstants.isBlueAlliance();
  }

  /**
   * Turret target from the latest robot state. Also refreshes the cached alliance, so it has to run
   * once per loop while the turret is aiming.
   *
   * @return Robot-Relative angle of the optimal yaw
   */
  private double getTurretTowardsGoalFromRobotPose() {
    refreshAlliance();
    return computeTurretTarget(
        RobotStateYearly.get().getLatestFieldRobotPose(),
        RobotStateYearly.get().getLatestMeasuredFieldRelativeChassisSpeeds());
  }

  /**
   * @return a command that sets the turret goal angle to the optimal shooting yaw
   */
  public Command createTurretTowardsGoalCommand() {
    return turret.smartPositionSetpointCommand(() -> getTurretTowardsGoalFromRobotPose());
  }

  /**
   * @return a command that aims the turret from the drivetrain odometry thread, falling back to the
   *     per-loop aim when no drivetrain states arrive
   */
  public Command createTurretFastAimCommand() {
    return turret.fastAimCommand(this::getTurretTowardsGoalFromRobotPose);
  }

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return True if turret is within 10 degrees of goal position, false otherwise
   */
  public boolean isTurretNearGoal() {
    double tolerance = RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig().kAutoAimLeniance;
    return MathUtil.isNear(
        turret.getCurrentPosition(), getTurretTowardsGoalFromRobotPose(), tolerance);
  }

  /**
   * @return A command to increase the turret offset
   */
  public Command createTurretOffsetIncreaseCommand() {
    // return new RunCommand(() -> turretOffset += 0.1);
    return new InstantCommand();
  }

  /**
   * @return A command to increase the turret offset
   */
  public Command createTurretOffsetDecreaseCommand() {
    return new InstantCommand();
  }

  public Command createTurretGoManualCommand() {
    return new InstantCommand(
        () -> {
          this.turretManual = true;
        });
  }

  public Command createTurretGoAutoCommand() {
    return new InstantCommand(
        () -> {
          this.turretManual = false;
        });
  }

  /* ---- FLYWHEEL COMMANDS ---- */

  /**
   * @return a command that sets the flywheel goal velocity to the optimal shooting velocity
   */
  public Command createFlywheelGoalSpeedCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> getCurrentSpeed());
  }

  /**
   * @return a command that sets the flywheel to the idle speed
   */
  public Command createFlywheelIdleSpeedCommand() {
    return flywheel.smartVelocitySetpointCommand(() -> 5);
  }

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return True if the flywheel is within 2 meters per second of goal velocity, false otherwise
   */
  public boolean isFlywheelNearGoal() {
    double tolerance =
        RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration().kAutoAimLeniance;
    return MathUtil.isNear(flywheel.getCurrentVelocity(), getCurrentSpeed(), tolerance);
  }

  /* ---- FUEL SHOOTING FUNCTIONS ---- */

  /**
   * Exists to prevent us from wasting fuel and from shooting fuel out of the field
   *
   * @return true if shooter is near goal position, see subsystem specific commands for actual
   *     values
   */
  public boolean isShooterNearGoal() {
    boolean yes = isFlywheelNearGoal() && isHoodNearGoal() && isTurretNearGoal();
    AEMLogger.recordOutput("IsShooterNearGoal", yes);
    // return yes;
    return true;
  }

  /**
   * @return A command that begins the fuel shooting process
   */
  public Command createShootFuelCommand() {

    return new ParallelCommandGroup(
        createFlywheelGoalSpeedCommand(), createHoodTowardsGoalCommand());
  }

  /* ---- SUPPLIER FUNCTIONS ---- */

  /**
   * @return A command that aims from the tower shot pose while it runs
   */
  public Command createSetPoseSupplierToTowerCommand() {
    return new RunCommand(() -> towerShotActive = true).finallyDo(() -> towerShotActive = false);
  }
}
