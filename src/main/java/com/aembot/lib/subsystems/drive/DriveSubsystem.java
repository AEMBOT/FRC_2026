package com.aembot.lib.subsystems.drive;

import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import com.aembot.lib.subsystems.drive.io.DrivetrainIO;
import com.aembot.lib.subsystems.drive.io.DrivetrainSimIO;
import com.aembot.lib.subsystems.drive.simulation.MapleSimSwerveDrivetrain;
import com.aembot.lib.subsystems.drive.visualizations.SwerveVisualizer;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends AEMSubsystem {
  protected final RobotState robotStateInstance;

  protected DrivetrainIO io;
  protected DrivetrainInputs inputs = new DrivetrainInputs();

  protected final DrivetrainConfiguration config;

  private final SwerveVisualizer visualizer;

  public DriveSubsystem(
      DrivetrainConfiguration configuration,
      DrivetrainIO drivetrain,
      RobotState robotStateInstance) {
    super(configuration.configurationName); // Logging prefix setup

    visualizer = new SwerveVisualizer(configuration.maxDriveSpeed);

    this.config = configuration;
    this.io = drivetrain;
    this.robotStateInstance = robotStateInstance;
  }

  /** Call {@link DriveSubsystem#resetPose} and return self */
  public DriveSubsystem withSetPose(Pose2d pose) {
    this.resetPose(pose);
    return this;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.updateInputs(inputs);

    updateLog();
    updateRobotState();
    io.logModules(inputs, logPrefixStandard);

    // Update standard deviations based on enable state
    if (DriverStation.isDisabled()) {
      configureStandardDevsForDisabled();
    } else {
      configureStandardDevsForEnabled();
    }

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    Logger.recordOutput(
        logPrefixStandard + "/CurrentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  /** Update the robot state with odometry info */
  private void updateRobotState() {
    // Use the timestamp logged with the data rather than the current timestamp.
    double timestamp = inputs.timestampRIOSynchronized;

    double rollRadsPerS = Units.degreesToRadians(inputs.rollAngularVelocity);
    double pitchRadsPerS = Units.degreesToRadians(inputs.pitchAngularVelocity);
    double yawRadsPerS = Units.degreesToRadians(inputs.yawAngularVelocity);
    double pitchRads = Units.degreesToRadians(inputs.pitch);
    double rollRads = Units.degreesToRadians(inputs.roll);

    /** Chassis speeds from drive encoders */
    ChassisSpeeds actualRobotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(inputs.Speeds, inputs.Pose.getRotation());

    /** Chassis speeds from drivetrain encoders + gyro */
    ChassisSpeeds gyroFusedFieldRelative =
        new ChassisSpeeds(
            inputs.Speeds.vxMetersPerSecond, inputs.Speeds.vyMetersPerSecond, yawRadsPerS);

    ChassisSpeeds desiredRobotRelative = inputs.kinematics.toChassisSpeeds(inputs.ModuleTargets);

    ChassisSpeeds desiredFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotRelative, inputs.Pose.getRotation());

    robotStateInstance.addOdometryMeasurement(timestamp, inputs.Pose);

    robotStateInstance.addChassisMotionMeasurements(
        timestamp,
        rollRadsPerS,
        pitchRadsPerS,
        yawRadsPerS,
        pitchRads,
        rollRads,
        inputs.accelX,
        inputs.accelY,
        actualRobotRelative,
        inputs.Speeds,
        desiredRobotRelative,
        desiredFieldRelative,
        gyroFusedFieldRelative);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);

    Logger.recordOutput(
        standardPrefix + "/Odometry/SpeedMetersPerSecond",
        new Translation2d(inputs.Speeds.vxMetersPerSecond, inputs.Speeds.vyMetersPerSecond)
            .getNorm());

    // Update the swerve module states
    visualizer.updateSwerveState(inputs);
  }

  /**
   * Resets the drive train odometry to the given pose. Ie. setting robot pose to auto starting
   * position. Wraps {@link DrivetrainIO#resetOdometry}.
   */
  public void resetPose(Pose2d pose) {
    io.resetOdometry(pose);
  }

  /**
   * Start a continuous command to apply new swerve drive requests each loop
   *
   * @param request Supplier of SwerveRequests that will be used to command the drivetrain
   * @return The command applying the request
   */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return Commands.run(() -> setRequest(request.get()), this).withName("SwerveDriveRequest");
  }

  /**
   * Set a control request instantaneously
   *
   * @param request The swerve drive request to pass to the drivetrain
   */
  public void setRequest(SwerveRequest request) {
    io.setRequest(request);
  }

  /** Apply configured deadbands to the given input */
  protected ChassisSpeeds applyDeadbands(ChassisSpeeds input) {
    if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond)
        < config.chassisTranslationSpeedThreshold) {
      input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
    }

    if (Math.abs(input.omegaRadiansPerSecond) < config.chassisRotationalSpeedThreshold) {
      input.omegaRadiansPerSecond = 0.0;
    }
    return input;
  }

  protected void setOdometryStdDevs(OdometryStandardDevs stdDevs) {
    io.setOdometryStdDevs(stdDevs.xStdDev(), stdDevs.yStdDev(), stdDevs.rotStdDev());
  }

  /** Set odometry standard deviation for when the robot is disabled based on config */
  public void configureStandardDevsForDisabled() {
    setOdometryStdDevs(this.config.disabledOdometryStandardDevs);
  }

  /** Set odometry standard deviation for when the robot is enabled based on config */
  public void configureStandardDevsForEnabled() {
    setOdometryStdDevs(this.config.enabledOdometryStandardDevs);
  }

  /**
   * @return the {@link MapleSimSwerveDrivetrain} if in sim. Otherwise null.
   */
  public MapleSimSwerveDrivetrain getSimDrivetrain() {
    if (io instanceof DrivetrainSimIO) {
      return ((DrivetrainSimIO) io).getMapleSimDrive();
    }

    return null;
  }
}
