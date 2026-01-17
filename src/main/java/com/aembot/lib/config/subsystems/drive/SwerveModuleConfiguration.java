package com.aembot.lib.config.subsystems.drive;

import com.aembot.lib.config.wrappers.ConfigureSlot0Gains;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.interfaces.MotorIO.NeutralMode;
import com.aembot.lib.math.mechanics.MultistageGearBox;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Base configuration class used to describe the configuration for one swerve module
 *
 * @param <DC> Config type for the drive motor
 * @param <SC> Config type for the steer motor
 * @param <EC> Config type for the abs encoder
 */
public abstract class SwerveModuleConfiguration<
    DC extends ParentConfiguration,
    SC extends ParentConfiguration,
    SE extends ParentConfiguration> {
  protected final String moduleName;

  protected double wheelRadiusM = 0;

  /** The minimum voltage required for the drive motor to begin moving */
  protected double driveFrictionVoltage = 0;

  /** The minimum voltage required for the steer motor to begin moving */
  protected double steerFrictionVoltage = 0;

  /**
   * The rotational moment of inertia in the drive system (kg * m^2). represents rotational
   * resistance to acceleration
   */
  protected double driveInertia = 0;

  /**
   * The rotational moment of inertia in the steer system (kg * m^2). represents rotational
   * resistance to acceleration
   */
  protected double steerInertia = 0;

  /** Motor Gains configured for the drive swerve motors */
  protected ConfigureSlot0Gains driveMotorGains = null;

  /** Motor Gains configured for the steer swerve motors */
  protected ConfigureSlot0Gains steerMotorGains = null;

  /** Offset from what the encoder thinks is 0 to the true zero of the module in rotations */
  protected double steerEncoderOffsetRotations = 0;

  /**
   * The amount of current (amps) that this motor is allowed to pull from the battery, if exceeded
   * voltage will be reduced to avoid brownouts
   */
  protected double driveMotorSupplyCurrentLimit = 0;

  /** The amount of current (amps) that motor is allowed to draw up to */
  protected double driveMotorStatorCurrentLimit = 0;

  /**
   * The amount of current (amps) that can be applied to the drive wheel before it slips (120
   * basically means it doesn't slip)
   */
  protected double driveMotorSlipCurrent = 0;

  protected DC driveMotorConfiguration = null;
  protected NeutralMode driveNeutralMode = NeutralMode.BRAKE;
  protected boolean driveMotorInverted = false;

  /**
   * The amount of current (amps) that this motor is allowed to pull from the battery, if exceeded
   * voltage will be reduced to avoid brownouts
   */
  protected double steerMotorSupplyCurrentLimit = 0;

  /** The amount of current (amps) that motor is allowed to draw up to */
  protected double steerMotorStatorCurrentLimit = 0;

  protected SC steerMotorConfiguration = null;
  protected boolean steerMotorInverted = false;
  protected NeutralMode steerNeutralMode = NeutralMode.BRAKE;

  protected SE steerEncoderConfiguration = null;
  protected boolean steerEncoderInverted = false;

  /** Gearbox between the drive motor and drive wheel of this swerve module */
  protected MultistageGearBox driveGearBox = null;

  /** Gearbox between the steer motor and the drive wheel of this swerve module */
  protected MultistageGearBox steerGearBox = null;

  protected CANDeviceID driveMotorID = null;
  protected CANDeviceID steerMotorID = null;
  protected CANDeviceID steerEncoderID = null;

  /** Position of the module on the robot */
  protected Translation2d locationOffset = null;

  /** Theoretical max speed of the module with no load in m/s. Automatically set when get */
  protected double maxRobotSpeedMeterPerSecond = Double.NaN;

  /**
   * specifies the {@link DCMotor} model that represents the physical characteristics of the drive
   * motor, including nominal voltage, free speed, stall torque, and current draw.
   */
  protected DCMotor driveMotorType = null;

  // Swerve constants
  protected SwerveModuleConstants<DC, SC, SE> ctreModuleConstants = null;

  public SwerveModuleConfiguration(
      String moduleName,
      CANDeviceID driveMotorID,
      CANDeviceID steerMotorID,
      CANDeviceID steerEncoderID) {
    this.moduleName = moduleName;
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.steerEncoderID = steerEncoderID;
  }

  /**
   * Set the minimum voltage required to move the swerve drive wheel
   *
   * @param voltage Voltage to be used achieve movement
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveFrictionVoltage(double voltage) {
    this.driveFrictionVoltage = voltage;
    return this;
  }

  /**
   * Set the minimum voltage required to turn the swerve steer wheel
   *
   * @param voltage Steer voltage to be used
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerFrictionVoltage(double voltage) {
    this.steerFrictionVoltage = voltage;
    return this;
  }

  /**
   * Set the rotational inertia for the drive system. Represents the rotational resistance to
   * acceleration (in kg·m²).
   *
   * @param inertia Rotational inertia for the drive system (kg·m²)
   * @return The SwerveModuleConfiguration being configured
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveInertia(double inertia) {
    this.driveInertia = inertia;
    return this;
  }

  /**
   * Set the rotational inertia for the steer system. Represents the rotational resistance to
   * acceleration (in kg·m²).
   *
   * @param inertia Rotational inertia for the steer system (kg·m²)
   * @return The SwerveModuleConfiguration being configured
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerInertia(double inertia) {
    this.steerInertia = inertia;
    return this;
  }

  /**
   * Set the PID and feedforward gains for the drive motor's Slot0 controller.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kV Velocity feedforward gain. Compensates for expected velocity demand
   * @param kS Static friction feedforward gain — compensates for motor stiction (voltage needed to
   *     start motion)
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorGains(
      double kP, double kI, double kD, double kV, double kS, double kG, double kA) {
    return this.withDriveMotorGains(new ConfigureSlot0Gains(kP, kI, kD, kV, kS, kG, kA));
  }

  /**
   * Set the PID and feedforward gains for the drive motor's Slot0 controller.
   *
   * @param slot The configured slot 0 gains
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorGains(ConfigureSlot0Gains slot) {
    this.driveMotorGains = slot;
    return this;
  }

  /**
   * Set the radius of the wheel in meters
   *
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withWheelRadiusM(double radiusM) {
    this.wheelRadiusM = radiusM;
    return this;
  }

  /**
   * Set the PID/FF gains for the steer motor's Slot0 controller.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kV Velocity feedforward gain - compensates for expected velocity demand
   * @param kS Static friction feedforward gain - compensates for motor stiction (voltage needed to
   *     start motion)
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerMotorGains(
      double kP, double kI, double kD, double kV, double kS, double kG, double kA) {
    return this.withSteerMotorGains(new ConfigureSlot0Gains(kP, kI, kD, kV, kS, kG, kA));
  }

  /**
   * Set the PID and feedforward gains for the steer motor's Slot0 controller.
   *
   * @param slot The configured slot 0 gains
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerMotorGains(ConfigureSlot0Gains slot) {
    this.steerMotorGains = slot;
    return this;
  }

  /**
   * Set the steer absolute encoder offset for the module (in rotations). Offset = (true zero) -
   * (reported zero), in rotations.
   *
   * @param rotations Offset in rotations
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withEncoderOffsetRotations(double rotations) {
    this.steerEncoderOffsetRotations = rotations;
    return this;
  }

  /**
   * Set the supply current limit for the drive motor, in amps. If exceeded, voltage may be reduced
   * to prevent brownouts.
   *
   * @param amps Maximum allowed supply current (A)
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorSupplyCurrentLimit(double amps) {
    this.driveMotorSupplyCurrentLimit = amps;
    return this;
  }

  /**
   * Set the stator (motor) current limit for the drive motor, in amps. This is the maximum motor
   * current draw the controller will allow.
   *
   * @param amps Maximum allowed stator current (A)
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorStatorCurrentLimit(double amps) {
    this.driveMotorStatorCurrentLimit = amps;
    return this;
  }

  /**
   * Set the slip current threshold for the drive wheel, in amps. Above this current the wheel is
   * considered likely to slip.
   *
   * @param amps Slip current threshold (A)
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorSlipCurrent(double amps) {
    this.driveMotorSlipCurrent = amps;
    return this;
  }

  /**
   * Set the gearbox configuration between the drive motor and the drive wheel.
   *
   * <p>The drive gearbox defines the total gear reduction between the motor output and the wheel
   * rotation, and directly affects the relationship between motor speed, torque, and wheel linear
   * velocity.
   *
   * @param gearBox The {@link MultistageGearBox} instance representing the drive gearbox
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveGearBox(MultistageGearBox gearBox) {
    this.driveGearBox = gearBox;
    return this;
  }

  /**
   * Set the gearbox configuration between the steer motor and the steering output.
   *
   * <p>The steer gearbox defines the total reduction between the motor and the steering mechanism,
   * determining how motor rotations translate to wheel angle changes. A higher ratio provides finer
   * control but slower steering response.
   *
   * @param gearBox The {@link MultistageGearBox} instance representing the steer gearbox
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerGearBox(MultistageGearBox gearBox) {
    this.steerGearBox = gearBox;
    return this;
  }

  /**
   * Set the CAN device ID for the drive motor controller.
   *
   * <p>This identifies which CAN bus and device ID correspond to the drive motor.
   *
   * @param canDevice The {@link CANDeviceID} representing the drive motor
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotor(CANDeviceID canDevice) {
    this.driveMotorID = canDevice;
    return this;
  }

  /**
   * Set the CAN device ID for the steer motor controller.
   *
   * <p>This identifies which CAN bus and device ID correspond to the steer motor.
   *
   * @param canDevice The {@link CANDeviceID} representing the steer motor
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerMotor(CANDeviceID canDevice) {
    this.steerMotorID = canDevice;
    return this;
  }

  /**
   * Set the CAN device ID for the steer encoder.
   *
   * <p>The steer encoder provides the absolute angular position of the swerve module, used for
   * precise steering control and module zeroing.
   *
   * @param canDevice The {@link CANDeviceID} representing the steer encoder
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerEncoder(CANDeviceID canDevice) {
    this.steerEncoderID = canDevice;
    return this;
  }

  /**
   * Set the physical location offset of this swerve module relative to the robot center.
   *
   * <p>This represents the position of the module on the robot frame, measured in meters along the
   * X and Y axes from the robot's center. Positive X is forward, and positive Y is to the left when
   * viewed from above.
   *
   * <p>This value is used in kinematic and odometry calculations to properly determine wheel
   * velocity vectors, rotation about the robot center, and field-relative motion.
   *
   * @param x The X offset from the robot center in meters (forward is positive)
   * @param y The Y offset from the robot center in meters (left is positive)
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withLocationOffset(double x, double y) {
    this.locationOffset = new Translation2d(x, y);
    return this;
  }

  /**
   * Set the motor type used for the drive motor of this swerve module.
   *
   * <p>This specifies the {@link DCMotor} model that represents the physical characteristics of the
   * drive motor, including nominal voltage, free speed, stall torque, and current draw. The motor
   * type is typically used in feedforward calculations, simulation models, and to characterize
   * drivetrain performance.
   *
   * <p>Common examples include:
   *
   * <ul>
   *   <li>{@code DCMotor.getFalcon500(1)}
   *   <li>{@code DCMotor.getNEO(1)}
   *   <li>{@code DCMotor.getKrakenX60(1)}
   * </ul>
   *
   * @param motor The {@link DCMotor} instance representing the drive motor type
   * @return This {@link SwerveModuleConfiguration} for chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorType(DCMotor motor) {
    this.driveMotorType = motor;
    return this;
  }

  /**
   * Sets whether the drive motor for this swerve module should be inverted.
   *
   * <p>This controls the direction of positive output for the drive motor. Setting this to {@code
   * true} will invert the drive motor's direction so that positive commands cause it to spin in the
   * opposite direction.
   *
   * @param inverted {@code true} to invert the drive motor, {@code false} to use normal direction
   * @return This {@link SwerveModuleConfiguration} instance for method chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorInverted(boolean inverted) {
    this.driveMotorInverted = inverted;
    return this;
  }

  /**
   * Sets whether the steer motor for this swerve module should be inverted.
   *
   * <p>This affects the direction the steering motor rotates when commanding positive rotation.
   * Setting this to {@code true} reverses that direction.
   *
   * @param inverted {@code true} to invert the steer motor, {@code false} to use normal direction
   * @return This {@link SwerveModuleConfiguration} instance for method chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerMotorInverted(boolean inverted) {
    this.steerMotorInverted = inverted;
    return this;
  }

  /**
   * Sets whether the steer encoder for this swerve module should be inverted.
   *
   * <p>This controls the sign convention for the steer encoder readings. Setting this to {@code
   * true} reverses the encoder’s reported angle direction, which can be useful when the physical
   * encoder orientation differs between modules.
   *
   * @param inverted {@code true} to invert the steer encoder reading, {@code false} for normal
   *     orientation
   * @return This {@link SwerveModuleConfiguration} instance for method chaining
   */
  public SwerveModuleConfiguration<DC, SC, SE> withSteerEncoderInverted(boolean inverted) {
    this.steerEncoderInverted = inverted;
    return this;
  }

  public SwerveModuleConfiguration<DC, SC, SE> withDriveNeutralMode(NeutralMode mode) {
    this.driveNeutralMode = mode;
    return this;
  }

  public SwerveModuleConfiguration<DC, SC, SE> withSteerNeutralMode(NeutralMode mode) {
    this.steerNeutralMode = mode;
    return this;
  }

  /**
   * Retrieve the theoretical maximum linear speed of this swerve module in meters per second.
   *
   * <p>This value is typically derived from the free speed of the drive motor, the total drive gear
   * reduction, and the wheel diameter:
   *
   * <pre>
   *     maxSpeed = (motorFreeSpeedRPM / gearRatio) * (wheelCircumferenceMeters / 60)
   * </pre>
   *
   * It represents the fastest linear velocity the module could achieve under ideal (no-load)
   * conditions. In practice, real-world values will be lower due to drivetrain inefficiencies,
   * voltage drops, and traction limitations.
   *
   * @return The maximum attainable module speed in meters per second
   */
  public double getMaxSpeedMetersPerSecond() {
    // Calc if unset
    if (maxRobotSpeedMeterPerSecond == Double.NaN) {
      maxRobotSpeedMeterPerSecond =
          (Units.radiansPerSecondToRotationsPerMinute(driveMotorType.freeSpeedRadPerSec) / 60.0)
              * Math.PI
              * wheelRadiusM
              / driveGearBox.getTotalRatio();
    }
    return maxRobotSpeedMeterPerSecond;
  }

  public CANDeviceID getDriveMotorID() {
    return driveMotorID;
  }

  public CANDeviceID getSteerMotorID() {
    return steerMotorID;
  }

  public CANDeviceID getSteerEncoderID() {
    return steerEncoderID;
  }

  /**
   * Retrieve the complete set of constants configured for this swerve module. This includes drive,
   * steer, and encoder parameters specific to the module.
   *
   * @return A CTRE {@link SwerveModuleConstants} instance containing all configuration data
   */
  public abstract SwerveModuleConstants<DC, SC, SE> getCtreModuleConstants();

  /**
   * Retrieve the configuration object for the drive motor associated with this swerve module.
   * Contains all TalonFX or motor controller settings such as PID gains, limits, and inversion.
   *
   * @return The drive motor configuration object of type {@code DC}
   */
  public abstract DC getDriveMotorConfiguration();

  /**
   * Retrieve the configuration object for the steer motor associated with this swerve module.
   * Contains all TalonFX or motor controller settings such as PID gains, limits, and inversion.
   *
   * @return The steer motor configuration object of type {@code SC}
   */
  public abstract SC getSteerMotorConfiguration();

  /**
   * Retrieve the configuration object for the steer encoder associated with this swerve module.
   * Contains all sensor-related settings such as offsets, direction, and conversion factors.
   *
   * @return The steer encoder configuration object of type {@code SE}
   */
  public abstract SE getSteerEncoderConfiguration();
}
