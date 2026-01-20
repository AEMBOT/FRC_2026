package com.aembot.lib.core.motors.interfaces;

import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.MotorInputs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.Pair;

public interface MotorIO {
  public enum NeutralMode {
    COAST,
    BRAKE
  }

  public enum FollowDirection {
    INVERT,
    SAME;

    public static FollowDirection fromCTREAlignment(MotorAlignmentValue value) {
      switch (value) {
        case Opposed:
          return INVERT;
        case Aligned:
        default:
          return SAME;
      }
    }

    public MotorAlignmentValue toCTREAlignment() {
      switch (this) {
        case INVERT:
          return MotorAlignmentValue.Opposed;
        case SAME:
        default:
          return MotorAlignmentValue.Aligned;
      }
    }
  }

  /** Get the name given to this motor */
  public String getName();

  /**
   * Update the MotorInputs object passed in with the current inputs to the motor
   *
   * @param inputs The inputs that are currently applied to the motor
   * @return true on success false on failure
   */
  public boolean updateInputs(MotorInputs inputs);

  /**
   * Enable or disabled forward and backwards software limits
   *
   * @param forwardLimitEnabled Enable (true) or disable (false) the forward software limits
   * @param reversLimitEnabled Enable (true) or disable (false) the reverse software limits
   * @return true on success false on failure
   */
  public boolean setEnableSoftwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled);

  /**
   * Get the current enabled state of both forward and reverse software limits
   *
   * @return A pair of booleans, this first is the forwardLimitEnabled state and the second is the
   *     reverseLimitEnabled state
   */
  public Pair<Boolean, Boolean> getEnableSoftwareLimits();

  /**
   * Enable or disabled forward and backwards hardware limits
   *
   * @param forwardLimitEnabled Enable (true) or disable (false) the forward hardware limits from
   *     connected limit switches
   * @param reverseLimitEnabled Enable (true) or disable (false) the reverse hardware limits from
   *     connected limit switches
   * @return true on success false on failure
   */
  public boolean setEnableHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled);

  /**
   * Get the current enabled state of both forward and reverse hardware limits
   *
   * @return A pair of booleans, this first is the forwardLimitEnabled state and the second is the
   *     reverseLimitEnabled state
   */
  public Pair<Boolean, Boolean> getEnableHardwareLimits();

  /**
   * Enable or disable functionality to automatically zero the motor when we hit a hardware limit
   * switch.
   *
   * <p>The hardware limit switches will automatically be enabled based on arguments to this method,
   * but will NOT be automatically disabled.
   *
   * @param forwardLimitEnabled Enable (true) or disable (false) automatically zeroing when we hit a
   *     forward hardware limit switch
   * @param reverseLimitEnabled Enable (true) or disable (false) automatically zeroing when we hit a
   *     reverse hardware limit switch
   * @return true on success false on failure
   */
  public boolean setZeroOnHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled);

  /**
   * Get whether or not the motor will zero itself upon hitting a hardware limit switch
   *
   * @return A pair of booleans, this first is the forwardLimitEnabled state and the second is the
   *     reverseLimitEnabled state
   */
  public Pair<Boolean, Boolean> getZeroOnHardwareLimits();

  /**
   * Set this motor to do what a different motor with some other CAN id is doing at exactly the same
   * time, either exactly following it or doing the inverse of what it does.
   *
   * @param masterDevice CAN device that we are following
   * @param direction The direction we are following the device at (Inverted or same)
   * @return true on success false on failure
   */
  public boolean follow(CANDeviceID masterDevice, FollowDirection direction);

  /**
   * Configure motor with motion magic configuration
   *
   * @param config The motion magic config to apply for this motor
   */
  public void setSmartMotorConfig(MotionMagicConfigs config);

  /**
   * Configure the voltage output of this motor
   *
   * @param config The voltage configuration to use
   */
  public void setVoltageConfig(VoltageConfigs config);

  /* ------ Encoder Zeroing ------ */
  /**
   * Set the encoders new relative position Example:
   *
   * <p>- Reported Encoder Value: 21.0
   *
   * <p>- setCurrentEncoderPosition(0.0)
   *
   * <p>- Reported Encoder Value: 0.0
   *
   * <p>In the above example whatever position the encoder was reporting as 21 is now what it thinks
   * is zero. This is not preserved between robot restarts as relative encoder values are *relative*
   *
   * @param position The new position the encoder should be reporting
   * @return true on success false on failure
   */
  public boolean setCurrentEncoderPosition(double position);

  /**
   * Sets the current encoder location to be the new relative zero
   *
   * @return true on success false on failure
   */
  public default boolean zeroEncoderPosition() {
    return setCurrentEncoderPosition(0.0);
  }

  /* ------ Motor Control  ------ */
  /**
   * Apply a given number of volts to this motor
   *
   * @param volts The amount of volts desired on this motor
   * @return true on success false on failure
   */
  public boolean setVoltageOutput(double volts);

  /**
   * Define an amount of current we want to apply direclty to torque. Might be CTRE only.
   *
   * @param current Amount of current in Amps to be driven into torque
   * @return true on success false on failure
   */
  public boolean setTorqueCurrent(double current);

  /**
   * Apply a duty cycle to the motor with no feedback
   *
   * @param dutyCycle Value between -1 and 1 to represent full reverse and full forward respectively
   * @return true on success false on failure
   */
  public boolean setOpenLoopDutyCycle(double dutyCycle);

  /**
   * Set how the motor behaves when no input is applied
   *
   * @param mode The mode that this motor should be in when no input is applied
   * @return true on success false on failure
   */
  public boolean setNeutralMode(NeutralMode mode);

  /**
   * Drive the motor to some position using standard PID / FF
   *
   * @param positionUnits The position with which the motor should be driven to (these units are
   *     relative to whatever config is used on this motor)
   * @return true on success false on failure
   */
  public boolean setPIDPositionSetpoint(double positionUnits, int slot);

  /**
   * Drive the motor to at some velocity using standard PID / FF
   *
   * @param posSetpoint The velocity with which the motor should be driven to (these units are
   *     relative to whatever config is used on this motor)
   * @return true on success false on failure
   */
  public boolean setPIDVelocitySetpoint(double velocityUnitsPerSecond, int slot);

  /**
   * Define a position set point for the motor to drive to with a "smart" control system such as
   * CTRE's Motion Magic or REV's Smart Motion.
   *
   * @param positionUnits The position with which the motor should be driven to (these units are
   *     relative to whatever config is used on this motor)
   * @param slot The slot in the gains bank to use to drive the position
   * @return true on success false on failure
   */
  public boolean setSmartPositionSetpoint(double positionUnits, int slot);

  /**
   * Define a position set point for the motor to drive to with a "smart" control system such as
   * CTRE's Motion Magic or REV's Smart Motion.
   *
   * <p>Assume slot 0 of the gains bank
   *
   * @param posSetpoint The position with which the motor should be driven to (these units are
   *     relative to whatever config is used on this motor)
   * @return true on success false on failure
   */
  public default boolean setSmartPositionSetpoint(double positionUnits) {
    return setSmartPositionSetpoint(positionUnits, 0);
  }
  ;

  /**
   * Define a position set point for the motor to drive to with a "smart" control system such as
   * CTRE's Motion Magic or REV's Smart Motion, with realtime updating of profiling gains.
   *
   * <p>This feature may only exist on CTRE Talon devices (unsure)
   *
   * @param positionUnits The position with which the motor should be driven to (these units are
   *     relative to whatever config is used on this motor)
   * @param velocity Velocity at which we wish to drive the motor at (rps)
   * @param acceleration Acceleration at which we wish to drive the motor (rps^2)
   * @param jerk The rate at which we can accelerate (rps^3)
   * @param feedforward The feedforward gains for the motor in volts
   * @param slot Gain slot to use on the device to drive the motion profile
   * @return true on success false on failure
   */
  public boolean setDynamicSmartPositionSetpoint(
      double positionUnits,
      double velocity,
      double acceleration,
      double jerk,
      double feedforward,
      int slot);

  /**
   * Define a position set point for the motor to drive to with a "smart" control system such as
   * CTRE's Motion Magic or REV's Smart Motion, with realtime updating of profiling gains.
   *
   * <p>This feature may only exist on CTRE Talon devices (unsure)
   *
   * <p>Assume slot 0 of the gains bank
   *
   * @param positionUnits The position with which the motor should be driven to (these units are
   *     relative to whatever config is used on this motor)
   * @param velocity Velocity at which we wish to drive the motor at (rps)
   * @param acceleration Acceleration at which we wish to drive the motor (rps^2)
   * @param jerk The rate at which we can accelerate (rps^3)
   * @param feedforward Feedforward to apply in volts to the motor
   * @return true on success false on failure
   */
  public default boolean setDynamicSmartPositionSetpoint(
      double positionUnits, double velocity, double acceleration, double jerk, double feedforward) {
    return setDynamicSmartPositionSetpoint(
        positionUnits, velocity, acceleration, jerk, feedforward, 0);
  }

  /**
   * Define a velocity set point for the motor to drive at with a "smart" control system such as
   * CTRE's Motion Magic or REV's Smart Motion
   *
   * @param unitsPerSecond The velocity setpoint with which the motor should be driven
   * @param slot The slot in the gains bank to use to drive the motor to the desired velocity and
   *     maintain it
   * @return true on success false on failure
   */
  public boolean setSmartVelocitySetpoint(double unitsPerSecond, int slot);

  /**
   * Define a velocity set point for the motor to drive at with a "smart" control system such as
   * CTRE's Motion Magic or REV's Smart Motion
   *
   * <p>Assume slot 0 of the gains bank
   *
   * @param unitsPerSecond The velocity setpoint with which the motor should be driven
   * @return true on success false on failure
   */
  public default boolean setSmartVelocitySetpoint(double unitsPerSecond) {
    return setSmartVelocitySetpoint(unitsPerSecond, 0);
  }
}
