package com.aembot.lib.subsystems.drive.commands;

import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class JoystickDriveCommand extends Command {
  private final DriveSubsystem drivetrainSubsystem;

  /** x joystick input; velocity on the axis parallel to the alliance wall */
  private final DoubleSupplier xVelocitySupplier;

  /** y joystick input; velocity on the axis perpendicular to the alliance wall */
  private final DoubleSupplier yVelocitySupplier;

  /** θ joystick input; steering. Its output may be null when driving with heading */
  private final Supplier<Double> steerVelocitySupplier;

  /**
   * The set heading we want to face. Its output may be null when driving with steer. If this is not
   * null, we'll drive with heading even if steer supplier is not null.
   */
  private final Supplier<Rotation2d> headingSupplier;

  private final double maxDriveSpeed;
  private final double maxAngularRate;

  private final double joystickSteerDeadband;
  private final double joystickDriveDeadband;

  private final SwerveRequest.FieldCentric driveWithSteerRequest;

  private final SwerveRequest.FieldCentricFacingAngle driveWithHeadingRequest;

  /**
   * Create a {@link JoystickDriveCommand} that consumes either an angular velocity or a heading. If
   * headingSupplier is not null, the robot will be driven with that heading. Other wise
   * steerVelocitySupplier will be used.
   *
   * @param subsystem The subsystem to control
   * @param driveTrainConfiguration The configuration for that subsystem
   * @param xVelocitySupplier Supplier for desired field-centric x velocity
   * @param yVelocitySupplier Supplier for desired field-centric y velocity
   * @param steerVelocitySupplier Supplier for desired angular velocity in radians per second
   * @param headingSupplier Supplier for desired robot heading
   * @see JoystickDriveCommand#createCommandWithSteer Factory for a command that only controls with
   *     steering
   * @see JoystickDriveCommand#createCommandWithHeading Factory for a command that only controls
   *     with heading
   */
  public JoystickDriveCommand(
      DriveSubsystem subsystem,
      DrivetrainConfiguration driveTrainConfiguration,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      Supplier<Double> steerVelocitySupplier,
      Supplier<Rotation2d> headingSupplier) {
    this.drivetrainSubsystem = subsystem;
    addRequirements(this.drivetrainSubsystem); // Reserves the drive subsystem for this command

    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.steerVelocitySupplier = steerVelocitySupplier;
    this.headingSupplier = headingSupplier;

    this.maxDriveSpeed = driveTrainConfiguration.maxDriveSpeed;
    this.maxAngularRate = driveTrainConfiguration.maxAngularRate;
    this.joystickSteerDeadband = driveTrainConfiguration.steerJoystickDeadband;
    this.joystickDriveDeadband = driveTrainConfiguration.driveJoystickDeadband;

    // Field centric drive WITHOUT heading lock set
    driveWithSteerRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(
                driveTrainConfiguration.chassisTranslationSpeedThreshold
                    * driveTrainConfiguration.driveJoystickDeadband)
            .withRotationalDeadband(
                driveTrainConfiguration.chassisRotationalSpeedThreshold
                    * driveTrainConfiguration.steerJoystickDeadband)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    // Field centric drive WITH heading lock
    driveWithHeadingRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(
                driveTrainConfiguration.chassisTranslationSpeedThreshold
                    * driveTrainConfiguration.driveJoystickDeadband)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  }

  /**
   * Create a {@link JoystickDriveCommand} that consumes an angular velocity
   *
   * @param subsystem The subsystem to control
   * @param driveTrainConfiguration The configuration for that subsystem
   * @param xVelocitySupplier Supplier for desired field-centric x velocity
   * @param yVelocitySupplier Supplier for desired field-centric y velocity
   * @param steerVelocitySupplier Supplier for desired angular velocity in radians per second
   */
  public static JoystickDriveCommand createCommandWithSteer(
      DriveSubsystem subsystem,
      DrivetrainConfiguration driveTrainConfiguration,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier
          steerVelocitySupplier // Make user pass in a DoubleSupplier so it's not nullable
      ) {
    return new JoystickDriveCommand(
        subsystem,
        driveTrainConfiguration,
        xVelocitySupplier,
        yVelocitySupplier,
        () -> steerVelocitySupplier.getAsDouble(),
        () -> null);
  }

  /**
   * Create a {@link JoystickDriveCommand} that consumes an angular velocity
   *
   * @param subsystem The subsystem to control
   * @param driveTrainConfiguration The configuration for that subsystem
   * @param xVelocitySupplier Supplier for desired field-centric x velocity
   * @param yVelocitySupplier Supplier for desired field-centric y velocity
   * @param headingSupplier Supplier for the desired heading of the robot
   */
  public static JoystickDriveCommand createCommandWithHeading(
      DriveSubsystem subsystem,
      DrivetrainConfiguration driveTrainConfiguration,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      Supplier<Rotation2d> headingSupplier) {
    return new JoystickDriveCommand(
        subsystem,
        driveTrainConfiguration,
        xVelocitySupplier,
        yVelocitySupplier,
        () -> null,
        headingSupplier);
  }

  /**
   * Create a {@link JoystickDriveCommand} that can be controlled with an angular velocity or a
   * fixed heading.
   *
   * @param subsystem The subsystem to control
   * @param driveTrainConfiguration The configuration for that subsystem
   * @param xVelocitySupplier Supplier for desired field-centric x velocity
   * @param yVelocitySupplier Supplier for desired field-centric y velocity
   * @param steerVelocitySupplier Supplier for desired angular velocity in radians per second
   * @param headingSupplier Supplier for the desired heading of the robot
   * @param lockHeading If true, the robot will face in the direction supplied by headingSupplier.
   *     Otherwise, it'll rotate according to steerVelocitySupplier.
   */
  public static JoystickDriveCommand createCommandWithRotationSwitch(
      DriveSubsystem subsystem,
      DrivetrainConfiguration driveTrainConfiguration,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier steerVelocitySupplier,
      Supplier<Rotation2d> headingSupplier,
      BooleanSupplier lockHeading) {
    return new JoystickDriveCommand(
        subsystem,
        driveTrainConfiguration,
        xVelocitySupplier,
        yVelocitySupplier,
        () -> lockHeading.getAsBoolean() ? null : steerVelocitySupplier.getAsDouble(),
        () -> lockHeading.getAsBoolean() ? headingSupplier.get() : null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xInput = this.xVelocitySupplier.getAsDouble();
    double yInput = this.yVelocitySupplier.getAsDouble();
    Double steerInput = this.steerVelocitySupplier.get();
    Rotation2d heading = this.headingSupplier.get();

    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    double xVelocity = 0;
    double yVelocity = 0;
    double steerVelocity = 0;

    if (Math.abs(xInput) > this.joystickDriveDeadband
        || Math.abs(yInput) > this.joystickDriveDeadband) {
      xVelocity = this.xVelocitySupplier.getAsDouble() * this.maxDriveSpeed;
      yVelocity = this.yVelocitySupplier.getAsDouble() * this.maxDriveSpeed;

      xVelocity = isRedAlliance ? -xVelocity : xVelocity;
      yVelocity = isRedAlliance ? -yVelocity : yVelocity;
    }

    if (steerInput != null && Math.abs(steerInput) > this.joystickSteerDeadband) {
      steerVelocity = -this.steerVelocitySupplier.get() * this.maxAngularRate;
    }

    if (heading == null) {
      driveWithSteer(xVelocity, yVelocity, steerVelocity);
    } else {
      driveWithHeading(xVelocity, yVelocity, heading);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void driveWithSteer(double velocityX, double velocityY, double velocityTheta) {
    driveWithSteerRequest.withVelocityX(velocityX).withVelocityY(velocityTheta);

    if (Math.abs(velocityTheta) > this.joystickSteerDeadband) {
      driveWithSteerRequest.withRotationalRate(velocityTheta);
    } else {
      driveWithSteerRequest.withRotationalRate(0);
    }

    drivetrainSubsystem.setRequest(driveWithSteerRequest);
  }

  private void driveWithHeading(double velocityX, double velocityY, Rotation2d heading) {
    drivetrainSubsystem.setRequest(
        driveWithHeadingRequest
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withTargetDirection(heading));
  }
}
