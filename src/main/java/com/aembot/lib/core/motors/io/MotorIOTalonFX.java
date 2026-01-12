package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.interfaces.CANable;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.factories.TalonFXFactory;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.phoenix6.CTREUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOTalonFX implements MotorIO, CANable {
  protected final TalonFX talon;

  private final MotorConfiguration<TalonFXConfiguration> config;

  /** Object to drive output using a duty cycle control */
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  /** Target velocity controller using voltage PID and FF */
  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);

  /** Target velocity controller using motion magic */
  private final MotionMagicVelocityVoltage motionMagicVelocityControl =
      new MotionMagicVelocityVoltage(0.0);

  /** Target position controller using voltage PID and FF */
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);

  /** Direct voltage controller that will attempt to drive the motor at a set voltage */
  private final VoltageOut voltageControl = new VoltageOut(0.0);

  /** Target position controller using motion magic */
  private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);

  /**
   * Target position controller using motion magic, allowing runtime modification to Cruise
   * Velocity, Acceleration, and Jerk
   */
  private final DynamicMotionMagicVoltage dynamicMotionMagicPositionControl =
      new DynamicMotionMagicVoltage(0.0, 0.0, 0.0, 0.0);

  /** Controller used to mimic the output of another TalonFX */
  private final Follower followerControl = new Follower(0, true);

  /** Controller to drive a motor to a desired stator current */
  private final TorqueCurrentFOC torqueCurrentFOCControl = new TorqueCurrentFOC(0.0);

  /**
   * Signal used to retreive current position values from the motor. This could be the position of a
   * remote sensor (ie. CANCoder) that we are using as feedback
   */
  private final StatusSignal<Angle> positionSignal;

  /** Signal used to retrieve current velocity values from the motor */
  private final StatusSignal<AngularVelocity> velocitySignal;

  /** Signal used to retreive current voltage values from the motor */
  private final StatusSignal<Voltage> voltageSignal;

  /** Signal used to retreive the current stator current applied to the motor */
  private final StatusSignal<Current> currentStatorSignal;

  /** Signal used to retrieve the current supply current applied to the motor */
  private final StatusSignal<Current> currentSupplySignal;

  /**
   * Signal used to retrieve the current rotor position of the motor (likely the same as
   * positionSignal unless using remote sensors)
   */
  private final StatusSignal<Angle> rotorPositionSignal;

  /** All the status signals we are using for this TalonFX */
  private final BaseStatusSignal[] signals;

  private MotorIOTalonFX(
      TalonFX talonFX, MotorConfiguration<TalonFXConfiguration> servoMotorConfig) {
    this.talon = talonFX;
    this.config = servoMotorConfig;

    // Set signal sources
    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();
    rotorPositionSignal = talon.getRotorPosition();

    // Setup list of signals to update all at once
    signals =
        new BaseStatusSignal[] {
          positionSignal,
          velocitySignal,
          voltageSignal,
          currentStatorSignal,
          currentSupplySignal,
          rotorPositionSignal
        };

    // Set the update frequency for all the signals
    CTREUtil.setUpdateFrequencyForAll(50.0, signals, talon.getDeviceID());

    // Optimization the bus utilization for the talon
    CTREUtil.Configuration.Motors.optimizeBusUtilization(talon);
  }

  /**
   * Construct a new instance of the TalonFXIO device with a {@link MotorConfiguration}
   *
   * @param device The CAN device that represents this motor
   * @param config The configuration used to determine how the motor should be driven outside the
   *     context of just the motor
   */
  public MotorIOTalonFX(MotorConfiguration<TalonFXConfiguration> config) {
    this(TalonFXFactory.createRawWithConfig(config.canDevice, config.getMotorConfig()), config);
  }

  /**
   * Construct a new instance of the TalonFXIO device with a {@link TalonFXConfiguration}
   *
   * @param device The CAN device that represents this motor
   * @param motorConfiguration The TalonFXConfiguration config used to determine how the motor
   *     should be driven
   */
  public MotorIOTalonFX(CANDeviceID device, TalonFXConfiguration motorConfiguration) {
    this(
        new MotorConfiguration<TalonFXConfiguration>()
            .withConfig(motorConfiguration)
            .withCANDevice(device));
  }

  /**
   * Construct a new instance of the TalonFXIO around a talon fx that already has a {@link
   * TalonFXConfiguration} attached.
   *
   * <p><b>NOTE</b>: This will result in a null {@link MotorConfiguration}, meaning many methods
   * will throw an exception. If a method requires the {@link MotorConfiguration}, that should be
   * stated in its documentation.
   *
   * <p>This should pretty much only be used on the drivetrain
   *
   * @param motor The TalonFX to use with this wrapper
   */
  public MotorIOTalonFX(TalonFX motor) {
    this(motor, null);
  }

  /**
   * Throws an exception if config is not set. Used to give a decent error message if we try to use
   * a method that requires config.
   *
   * @throws NullPointerException if config is null
   */
  private void checkServoMotorConfig() {
    if (this.config == null) {
      throw new NullPointerException(
          "Tried to use a method that requires a set MotorConfiguration config when config is null. "
              + "If this method is needed, try a MotorIOTalonFX constructor that initializes config.");
    }
  }

  /** Get a reference to the wrapped {@link TalonFX} */
  public TalonFX getTalon() {
    return talon;
  }

  @Override
  public CANDeviceID getCANDevice() {
    checkServoMotorConfig();
    return this.config.canDevice;
  }

  @Override
  public String getName() {
    checkServoMotorConfig();
    return getCANDeviceName();
  }

  @Override
  public boolean updateInputs(MotorInputs inputs) {
    StatusCode refreshStatus = BaseStatusSignal.refreshAll(signals);

    inputs.positionUnits = getRotorRotationsToUnits(positionSignal.getValueAsDouble());
    inputs.velocityUnitsPerSecond = getRotorRotationsToUnits(velocitySignal.getValueAsDouble());
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    inputs.rawRotorPosition = rotorPositionSignal.getValueAsDouble();

    return refreshStatus == StatusCode.OK;
  }

  /**
   * Convert the given rotor rotations to the real-world units specified in config
   *
   * @param rotorRotations Rotor rotation value we want to convert into the in-use units
   * @return The rotations converted into some units as defined in the config
   */
  private double getRotorRotationsToUnits(double rotorRotations) {
    checkServoMotorConfig();
    return this.config.getRotationsToUnits(rotorRotations);
  }

  /**
   * Convert the current units into rotor rotations using the defined conversion ratio
   *
   * @param units Units we want to convert to rotor rotations
   * @return The resulting rotor rotations
   */
  private double getUnitsToRotorRotations(double units) {
    checkServoMotorConfig();
    return this.config.getUnitsToRotations(units);
  }

  /**
   * Clamp the given value and convert it to rotations
   *
   * @param units Units we want to convert to a clampped rotor position
   * @return Clamped rotor rotation
   */
  private double clampPosition(double units) {
    checkServoMotorConfig();
    return getUnitsToRotorRotations(
        MathUtil.clamp(units, this.config.minPositionUnits, this.config.maxPositionUnits));
  }

  /* ---- CONFIG ---- */

  @Override
  public boolean setNeutralMode(NeutralMode mode) {
    checkServoMotorConfig();

    switch (mode) {
      case BRAKE:
        this.config.getMotorConfig().MotorOutput.NeutralMode = NeutralModeValue.Brake;
        break;
      case COAST:
        this.config.getMotorConfig().MotorOutput.NeutralMode = NeutralModeValue.Coast;
        break;
    }

    return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
  }

  @Override
  public boolean setEnableSoftwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
    checkServoMotorConfig();

    this.config.getMotorConfig().SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnabled;
    this.config.getMotorConfig().SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnabled;
    return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
  }

  @Override
  public Pair<Boolean, Boolean> getEnableSoftwareLimits() {
    checkServoMotorConfig();

    return new Pair<Boolean, Boolean>(
        this.config.getMotorConfig().SoftwareLimitSwitch.ForwardSoftLimitEnable,
        this.config.getMotorConfig().SoftwareLimitSwitch.ReverseSoftLimitEnable);
  }

  @Override
  public boolean setEnableHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
    checkServoMotorConfig();

    this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnabled;
    this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnabled;
    return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
  }

  @Override
  public Pair<Boolean, Boolean> getEnableHardwareLimits() {
    checkServoMotorConfig();

    return new Pair<>(
        this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitEnable,
        this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitEnable);
  }

  @Override
  public boolean setZeroOnHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
    checkServoMotorConfig();

    this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
        forwardLimitEnabled;
    this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitAutosetPositionEnable =
        reverseLimitEnabled;

    if (forwardLimitEnabled) {
      this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnabled;
    }
    if (reverseLimitEnabled) {
      this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnabled;
    }
    return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
  }

  @Override
  public Pair<Boolean, Boolean> getZeroOnHardwareLimits() {
    checkServoMotorConfig();

    return new Pair<>(
        this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitAutosetPositionEnable,
        this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitAutosetPositionEnable);
  }

  @Override
  public void setSmartMotorConfig(MotionMagicConfigs config) {
    checkServoMotorConfig();

    this.config.getMotorConfig().MotionMagic = config;
    CTREUtil.Configuration.Motors.applyConfiguration(talon, config);
  }

  @Override
  public void setVoltageConfig(VoltageConfigs config) {
    checkServoMotorConfig();

    this.config.getMotorConfig().Voltage = config;
    CTREUtil.Configuration.Motors.applyConfigurationNonBlocking(talon, config);
  }

  @Override
  public boolean setCurrentEncoderPosition(double position) {
    return talon.setPosition(getUnitsToRotorRotations(position)) == StatusCode.OK;
  }

  /* ---- CONTROL ---- */

  @Override
  public boolean setOpenLoopDutyCycle(double dutyCycle) {

    return talon.setControl(dutyCycleControl.withOutput(dutyCycle)) == StatusCode.OK;
  }

  @Override
  public boolean setVoltageOutput(double volts) {
    return talon.setControl(voltageControl.withOutput(volts)) == StatusCode.OK;
  }

  @Override
  public boolean setPIDVelocitySetpoint(double velocity, int slot) {
    return talon.setControl(velocityVoltageControl.withVelocity(velocity).withSlot(slot))
        == StatusCode.OK;
  }

  @Override
  public boolean setPIDPositionSetpoint(double position, int slot) {
    return talon.setControl(
            positionVoltageControl.withPosition(clampPosition(position)).withSlot(slot))
        == StatusCode.OK;
  }

  @Override
  public boolean follow(CANDeviceID masterDevice, FollowDirection direction) {
    checkServoMotorConfig();

    this.getCANDevice().setMasterCANDevice(masterDevice);
    return talon.setControl(
            followerControl
                .withMasterID(masterDevice.getDeviceID())
                .withOpposeMasterDirection(direction == FollowDirection.INVERT))
        == StatusCode.OK;
  }

  @Override
  public boolean setTorqueCurrent(double current) {
    return talon.setControl(torqueCurrentFOCControl.withOutput(current)) == StatusCode.OK;
  }

  @Override
  public boolean setSmartPositionSetpoint(double position, int slot) {
    checkServoMotorConfig();

    MotionMagicVoltage mmVoltage =
        motionMagicPositionControl.withPosition(clampPosition(position)).withSlot(slot);

    return talon.setControl(mmVoltage) == StatusCode.OK;
  }

  @Override
  public boolean setDynamicSmartPositionSetpoint(
      double position,
      double velocity,
      double acceleration,
      double jerk,
      double feedforward,
      int slot) {
    return talon.setControl(
            dynamicMotionMagicPositionControl
                .withPosition(clampPosition(position))
                .withVelocity(velocity)
                .withAcceleration(acceleration)
                .withJerk(jerk)
                .withFeedForward(feedforward)
                .withSlot(slot))
        == StatusCode.OK;
  }

  @Override
  public boolean setSmartVelocitySetpoint(double velocity, int slot) {
    checkServoMotorConfig();

    return talon.setControl(
            motionMagicVelocityControl
                .withVelocity(getUnitsToRotorRotations(velocity))
                .withSlot(slot))
        == StatusCode.OK;
  }
}
