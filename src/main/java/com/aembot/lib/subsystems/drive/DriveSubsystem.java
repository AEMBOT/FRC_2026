package com.aembot.lib.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.aembot.lib.config.odometry.OdometryStandardDevs;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import com.aembot.lib.subsystems.drive.io.DrivetrainIO;
import com.aembot.lib.subsystems.drive.io.DrivetrainSimIO;
import com.aembot.lib.subsystems.drive.simulation.MapleSimSwerveDrivetrain;
import com.aembot.lib.subsystems.drive.visualizations.SwerveVisualizer;
import com.ctre.phoenix6.swerve.SwerveModule;
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
  private static final double MODULE_DETAIL_LOG_PERIOD_SECONDS = 0.1;
  private static final double CURRENT_COMMAND_LOG_PERIOD_SECONDS = 0.1;

  protected final RobotState robotStateInstance;

  protected DrivetrainIO io;
  protected DrivetrainInputs inputs = new DrivetrainInputs();

  protected final DrivetrainConfiguration config;

  private final SwerveVisualizer visualizer;
  private double nextModuleDetailLogTimestampSeconds = 0.0;
  private double nextCurrentCommandLogTimestampSeconds = 0.0;
  private String lastLoggedCurrentCommandName = "";
  private final String driveInputsLogKey;
  private final String driveOdometrySpeedLogKey;
  private final String driveLatencyPeriodicLogKey;
  private final String driveCurrentCommandLogKey;

  public DriveSubsystem(
      DrivetrainConfiguration configuration,
      DrivetrainIO drivetrain,
      RobotState robotStateInstance) {
    super(configuration.configurationName); // Logging prefix setup

    visualizer = new SwerveVisualizer(configuration.maxDriveSpeed);

    this.config = configuration;
    this.io = drivetrain;
    this.robotStateInstance = robotStateInstance;
    this.driveInputsLogKey = logPrefixInput;
    this.driveOdometrySpeedLogKey = logPrefixStandard + "/Odometry/SpeedMetersPerSecond";
    this.driveLatencyPeriodicLogKey = logPrefixStandard + "/LatencyPeriodicMS";
    this.driveCurrentCommandLogKey = logPrefixStandard + "/CurrentCommand";

    robotStateInstance.registerAprilCameraOutputConsumer(io::addVisionEstimation);
  }

  /** Call {@link DriveSubsystem#resetPose} and return self */
  public DriveSubsystem withSetPose(Pose2d pose) {
    this.resetPose(pose);
    return this;
  }

  @Override
  @Traced
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.updateInputs(inputs);

    updateLog();
    updateRobotState();
    if (timestamp >= nextModuleDetailLogTimestampSeconds) {
      nextModuleDetailLogTimestampSeconds = timestamp + MODULE_DETAIL_LOG_PERIOD_SECONDS;
      io.logModules(inputs, logPrefixStandard);
    }

    // Update standard deviations based on enable state
    if (DriverStation.isDisabled()) {
      configureStandardDevsForDisabled();
    } else {
      configureStandardDevsForEnabled();
    }

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(driveLatencyPeriodicLogKey, (Timer.getFPGATimestamp() - timestamp) * 1000);

    String currentCommandName =
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName();
    if (!currentCommandName.equals(lastLoggedCurrentCommandName)
        || timestamp >= nextCurrentCommandLogTimestampSeconds) {
      lastLoggedCurrentCommandName = currentCommandName;
      nextCurrentCommandLogTimestampSeconds = timestamp + CURRENT_COMMAND_LOG_PERIOD_SECONDS;
      Logger.recordOutput(driveCurrentCommandLogKey, currentCommandName);
    }
  }

  /** Update the robot state with odometry info */
  @Traced
  private void updateRobotState() {
    // Use the timestamp logged with the data rather than the current timestamp.
    double timestamp = inputs.timestampRIOSynchronized;

    double rollRadsPerS = Units.degreesToRadians(inputs.rollAngularVelocity);
    double pitchRadsPerS = Units.degreesToRadians(inputs.pitchAngularVelocity);
    double yawRadsPerS = Units.degreesToRadians(inputs.yawAngularVelocity);
    double pitchRads = Units.degreesToRadians(inputs.pitch);
    double rollRads = Units.degreesToRadians(inputs.roll);

    /** Chassis speeds from drive encoders */
    ChassisSpeeds actualRobotRelative = inputs.Speeds;

    ChassisSpeeds actualFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(actualRobotRelative, inputs.Pose.getRotation());

    /** Chassis speeds from drivetrain encoders + gyro */
    ChassisSpeeds gyroFusedFieldRelative =
        new ChassisSpeeds(
            actualFieldRelative.vxMetersPerSecond,
            actualFieldRelative.vyMetersPerSecond,
            yawRadsPerS);

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
        actualFieldRelative,
        desiredRobotRelative,
        desiredFieldRelative,
        gyroFusedFieldRelative);
  }

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(driveInputsLogKey, inputs);

    Logger.recordOutput(
        driveOdometrySpeedLogKey,
        new Translation2d(inputs.Speeds.vxMetersPerSecond, inputs.Speeds.vyMetersPerSecond)
            .getNorm());

    // Update the swerve module states
    visualizer.updateSwerveState(inputs);
  }

  /**
   * Resets the drive train odometry to the given pose. Ie. setting robot pose to auto starting
   * position. Wraps {@link DrivetrainIO#resetOdometry}.
   */
  @Traced
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
  @Traced
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

  /**
   * Create a new swerve request from a robot relative chassis speeds variable
   *
   * @param speeds robot relative chassis speeds
   */
  @Traced
  public void setRequestFromChassisSpeeds(ChassisSpeeds speeds) {
    setRequest(
        new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
  }

  @Traced
  public void setRequestFromSwerveSample(SwerveSample sample) {

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + config.autoTranslationController.calculate(inputs.Pose.getX(), sample.x),
            sample.vy + config.autoTranslationController.calculate(inputs.Pose.getY(), sample.y),
            sample.omega
                + config.autoRotationController.calculate(
                    inputs.Pose.getRotation().getRadians(), sample.heading));

    setRequestFromChassisSpeeds(speeds);
  }
}
