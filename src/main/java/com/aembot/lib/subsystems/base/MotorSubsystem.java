package com.aembot.lib.subsystems.base;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Flexible base subsystem for any {@link MotorIO}-driven mechanism that can behave like a servo. It
 * wraps a configurable motor controller, handles telemetry, and exposes commands for open-loop and
 * closed-loop position/velocity control.
 *
 * <p>Key capabilities provided by this class include:
 *
 * <ul>
 *   <li>Polling {@link MotorInputs} and pushing telemetry to {@link
 *       org.littletonrobotics.junction.Logger AKit's Logger}
 *   <li>Helper methods for managing encoder offsets and setpoints
 *   <li>Command factories for controlling with Motion Magic, PID, & duty-cycle commands
 * </ul>
 *
 * @param <I> {@link MotorInputs} class capturing feedback for this motor.
 * @param <M> {@link MotorIO} IO interface for the motor.
 * @param <C> {@link MotorConfiguration} describing behaviour of the motor
 */
public abstract class MotorSubsystem<
        I extends MotorInputs, M extends MotorIO, C extends MotorConfiguration<?>>
    extends AEMSubsystem {

  /** The {@link MotorIO} object that we are commanding */
  protected final M io;

  /** The {@link MotorInputs} object that we are using to keep the state of the motor */
  protected final I inputs;

  /** What {@link MotorConfiguration} is being used */
  protected final C motorConfig;

  /** The current position in servo motor configuration units of where this motor should be */
  protected double currentPositionSetpoint = 0;

  /**
   * Create new servo motor subsystem with the desired motor and motor config
   *
   * @param name The string name of the subsystem. Used for the logging prefix.
   * @param motorInputs The motor inputs object that this servo motor is using to track the state
   * @param motor The MotorIO type that this servo motor subsystem is driving
   * @param motorConfiguration The ServoMotorConfiguration that is in use with this system
   */
  public MotorSubsystem(String name, I motorInputs, M motor, C motorConfiguration) {
    super(name);
    this.motorConfig = motorConfiguration;
    this.inputs = motorInputs;
    this.io = motor;

    setDefaultCommand(
        dutyCycleCommand(() -> 0.0)
            .withName("DefaultNeutral")
            .ignoringDisable(true) // Do this even when disabled
        );
  }

  /**
   * Create new servo motor subsystem with the desired motor and motor config. Name defaults to that
   * of motorConfiguration.
   *
   * @param motorInputs The motor inputs object that this servo motor is using to track the state
   * @param motor The MotorIO type that this servo motor subsystem is driving
   * @param motorConfiguration The ServoMotorConfiguration that is in use with this system
   */
  public MotorSubsystem(I motorInputs, M motor, C motorConfiguration) {
    this(motorConfiguration.configurationName, motorInputs, motor, motorConfiguration);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateLog();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix + "/Inputs", inputs);

    Logger.recordOutput(
        standardPrefix + "/CurrentCommand",
        (getCurrentCommand() == null) ? "NONE" : getCurrentCommand().getName());
  }

  /* ---------  MOTOR-EMBEDDED ENCODER FUNCTIONALITY --------- */

  /**
   * Get the current position of this motor subsystem in the context of the defined units
   *
   * @return Current position of the relative encoder embedded in this motor
   */
  public double getCurrentPosition() {
    return inputs.positionUnits;
  }

  /**
   * Get the current velocity of the motor subsystem in the context of the defined units
   *
   * @return Current velocity of the motor in the context of the subsystem units
   */
  public double getCurrentVelocity() {
    return inputs.velocityUnitsPerSecond;
  }

  /**
   * Set the position of the motor's embedded encoder to the given value
   *
   * @param position The new position of the encoder in units
   */
  protected void setEncoderPosition(double position) {
    io.setCurrentEncoderPosition(position);
  }

  /** Set current encoder location to be the percieved zero of the motor-embedded encoder */
  protected void zeroEncoderPosition() {
    io.zeroEncoderPosition();
  }

  /* ---------  CONFIG --------- */

  protected void setSmartMotionConfigImpl(MotionMagicConfigs config) {
    io.setSmartMotorConfig(config);
  }

  protected void setNeutralModeImpl(MotorIO.NeutralMode mode) {
    Logger.recordOutput(logPrefixStandard + "/SetNeutralMode", mode.toString());
    io.setNeutralMode(mode);
  }

  /* ---------  CONTROL --------- */

  /**
   * Get the current position setpoint that this motor is attempting to reach
   *
   * @return The position setpoint in the subsystem units that we are trying to reach with the motor
   */
  public double getPositionSetpointUnits() {
    return currentPositionSetpoint;
  }

  // ---  CONTROL: Duty Cycle

  protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
    Logger.recordOutput(logPrefixStandard + "/SetOpenLoopDutyCycle", dutyCycle);
    io.setOpenLoopDutyCycle(dutyCycle);
  }

  // ---  CONTROL: Voltage

  protected void setVoltageImpl(double voltage) {
    Logger.recordOutput(logPrefixStandard + "/SetVoltage", voltage);
    io.setVoltageOutput(voltage);
  }

  // --- CONTROL: Torque

  protected void setTorqueCurrentImpl(double current) {
    Logger.recordOutput(logPrefixStandard + "/SetTorqueCurrent", current);
    io.setTorqueCurrent(current);
  }

  // --- CONTROL: PID Velocity

  protected void setPIDVelocitySetpointImpl(double velocity, int slot) {
    Logger.recordOutput(logPrefixStandard + "/SetPIDVelocitySetpoint", velocity);
    io.setPIDVelocitySetpoint(velocity, slot);
  }

  protected void setPIDVelocitySetpointImpl(double velocity) {
    setPIDVelocitySetpointImpl(velocity, 0);
  }

  // --- CONTROL: PID Position

  protected void setPIDPositionSetpointImpl(double position, int slot) {
    currentPositionSetpoint = position;
    Logger.recordOutput(logPrefixStandard + "/SetPIDPositionSetpoint", position);
    io.setPIDPositionSetpoint(position, slot);
  }

  protected void setPIDPositionSetpointImpl(double position) {
    setPIDPositionSetpointImpl(position, 0);
  }

  // --- CONTROL: Smart Positioning

  protected void setSmartPositionSetpointImpl(double position) {
    setSmartPositionSetpointImpl(position, 0);
  }

  protected void setSmartPositionSetpointImpl(double position, int slot) {
    currentPositionSetpoint = position;
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Position", position);
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Slot", slot);
    io.setSmartPositionSetpoint(position, slot);
  }

  // --- CONTROL: Dynamic Smart Positioning

  protected void setDynamicSmartPositionSetpointImpl(
      double position, MotionMagicConfigs config, int slot) {
    setDynamicSmartPositionSetpointImpl(
        position,
        config.MotionMagicCruiseVelocity,
        config.MotionMagicAcceleration,
        config.MotionMagicJerk,
        0.0,
        slot);
  }

  protected void setDynamicSmartPositionSetpointImpl(
      double position, double feedforward, MotionMagicConfigs config, int slot) {
    setDynamicSmartPositionSetpointImpl(
        position,
        config.MotionMagicCruiseVelocity,
        config.MotionMagicAcceleration,
        config.MotionMagicJerk,
        feedforward,
        slot);
  }

  protected void setDynamicSmartPositionSetpointImpl(
      double position,
      double velocity,
      double acceleration,
      double jerk,
      double feedforward,
      int slot) {
    currentPositionSetpoint = position;
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Position", position);
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Velocity", velocity);
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Acceleration", acceleration);

    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Jerk", jerk);
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Feedforward", feedforward);
    Logger.recordOutput(logPrefixStandard + "/SetSmartPositionSetpoint/Slot", slot);
    io.setDynamicSmartPositionSetpoint(position, velocity, acceleration, jerk, feedforward, slot);
  }

  // --- CONTROL: Dynamic Smart Velocity

  protected void setSmartVelocitySetpointImpl(double velocity, int slot) {
    Logger.recordOutput(logPrefixStandard + "/SetSmartVelocitySetpoint/Velocity", velocity);
    Logger.recordOutput(logPrefixStandard + "/SetSmartVelocitySetpoint/Slot", slot);
    io.setSmartVelocitySetpoint(velocity, slot);
  }

  protected void setSmartVelocitySetpointImpl(double velocity) {
    setSmartVelocitySetpointImpl(velocity, 0);
  }

  /* --------- COMMANDS --------- */

  // --- COMMANDS: Motor Config

  /**
   * Set the smart motion config for the given motor subsystem
   *
   * @param config The configuration we want to use for this given motor
   * @return reference to the command base object so we can chain
   */
  public Command smartMotionConfigCommand(MotionMagicConfigs config) {
    /*
     We don't want to want to require the current subsystem here because we don't *really*
     need the subsystem exclusively as we are just changing the config
    */
    return new InstantCommand(() -> setSmartMotionConfigImpl(config))
        .withName("SetSmartMotionConfig");
  }

  // --- COMMANDS: Control Modifiers

  /**
   * Disable software limits while this command is run, restoring them as they were before after it
   * terminates.
   */
  public Command withoutSoftwareLimitsTemporailyCommand() {
    // Needs to be final or else Java will throw a fit. Can't be a pair bcuz its fields are final.
    // bleh
    final var prevLimits =
        new Object() {
          boolean forwardLimitsEnabled = false;
          boolean reverseLimitsEnabled = false;

          void fromPair(Pair<Boolean, Boolean> limits) {
            this.forwardLimitsEnabled = limits.getFirst();
            this.reverseLimitsEnabled = limits.getSecond();
          }
        };

    // This doesn't have any requirements because it shouldn't require the subsystem
    return Commands.startEnd(
            () -> {
              prevLimits.fromPair(io.getEnableSoftwareLimits());
              io.setEnableSoftwareLimits(false, false);
            },
            () -> {
              io.setEnableSoftwareLimits(
                  prevLimits.forwardLimitsEnabled, prevLimits.reverseLimitsEnabled);
            })
        .withName("WithoutSoftwareLimitsTemp");
  }

  /**
   * Run the command passed in with the software limits disabled, renabling them once the command is
   * complete
   *
   * @param commandToRun The command we wish to run with the software limits enabled
   * @return The command to be run
   */
  public Command runWithoutSoftwareLimitsCommand(Command commandToRun) {
    return new ParallelDeadlineGroup(commandToRun, withoutSoftwareLimitsTemporailyCommand())
        .withName("Running_" + commandToRun.getName() + "_WithoutSoftwareLimitsTemp");
  }

  // --- COMMANDS: Neutral Mode

  /**
   * Set the neutral mode of the motor to be coast mode
   *
   * @return Reference to the command that was run
   */
  public Command setCoastCommand() {
    return new InstantCommand(
            () -> {
              setNeutralModeImpl(MotorIO.NeutralMode.COAST);
            })
        .withName("SetCoast");
  }

  /**
   * Set the neutral mode of the motor to be brake mode
   *
   * @return Reference to the command that was run
   */
  public Command setBrakeCommand() {
    return new InstantCommand(
            () -> {
              setNeutralModeImpl(MotorIO.NeutralMode.BRAKE);
            })
        .withName("SetBrake");
  }

  // COMMANDS: Duty Cycle Control

  /**
   * Command the motor to run at a given duty cycle
   *
   * @param dutyCycle Double supplier supplying the duty cycle (-1 to 1)
   * @return Reference to the command being run
   */
  public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
            },
            () -> {
              setOpenLoopDutyCycleImpl(0.0);
            })
        .withName("DutyCycleControl");
  }

  /**
   * Command the motor to run at a given duty cycle
   *
   * @param dutyCycle Double supplier supplying the duty cycle (-1 to 1)
   * @return Reference to the command being run
   */
  public Command dutyCycleCommandNoEnd(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
            },
            () -> {})
        .withName("DutyCycleControl");
  }

  // --- COMMANDS: Voltage Control

  /**
   * Drive the motor at a specified voltage until canceled
   *
   * @param voltage Voltage to drive the motor at
   * @return The command that is being run
   */
  public Command voltageCommand(DoubleSupplier voltage) {
    return runEnd(
            () -> {
              setVoltageImpl(voltage.getAsDouble());
            },
            () -> {
              setVoltageImpl(0.0);
            })
        .withName("VoltageControl");
  }

  // --- COMMANDS: Torque Control

  public Command torqueCurrentCommand(DoubleSupplier current) {
    return runEnd(
            () -> {
              setTorqueCurrentImpl(current.getAsDouble());
            },
            () -> {
              setTorqueCurrentImpl(0.0);
            })
        .withName("TorqueCurrent");
  }

  // --- COMMANDS: Velocity Control

  public Command velocitySetpointCommand(DoubleSupplier velocity) {
    return runEnd(
            () -> {
              setPIDVelocitySetpointImpl(velocity.getAsDouble());
            },
            () -> {})
        .withName("PIDVelocityControl");
  }

  public Command smartVelocitySetpointCommand(DoubleSupplier velocity, int slot) {
    return runEnd(
            () -> {
              setSmartVelocitySetpointImpl(velocity.getAsDouble(), slot);
            },
            () -> {})
        .withName("SmartVelocityControl");
  }

  public Command smartVelocitySetpointCommand(DoubleSupplier velocity) {
    return smartVelocitySetpointCommand(velocity, 0);
  }

  // --- COMMANDS: PID Position Control

  public Command positionSetpointCommand(DoubleSupplier position, int slot) {
    return runEnd(
            () -> {
              setPIDPositionSetpointImpl(position.getAsDouble(), slot);
            },
            () -> {})
        .withName("PIDPositionControl");
  }

  public Command positionSetpointUntilOnTargetCommand(
      DoubleSupplier position, DoubleSupplier acceptableError, int slot) {
    return positionSetpointCommand(position, slot)
        .until(
            () ->
                MathUtil.isNear(
                    position.getAsDouble(), inputs.positionUnits, acceptableError.getAsDouble()))
        .withName("PIDPositionControlUntilOnTarget");
  }

  public Command positionSetpointUntilOnTargetCommand(
      DoubleSupplier position, DoubleSupplier acceptableError) {
    return positionSetpointUntilOnTargetCommand(position, acceptableError, 0);
  }

  // --- COMMANDS: Smart Position Control

  public Command smartPositionSetpointCommand(DoubleSupplier position, int slot) {
    return runEnd(
            () -> {
              setSmartPositionSetpointImpl(position.getAsDouble(), slot);
            },
            () -> {})
        .withName("SmartPositionControl");
  }

  public Command smartPositionSetpointCommand(DoubleSupplier position) {
    return smartPositionSetpointCommand(position, 0);
  }

  public Command smartPositionSetpointUntilOnTargetCommand(
      DoubleSupplier position, DoubleSupplier acceptableError, int slot) {
    return smartPositionSetpointCommand(position, slot)
        .until(
            () ->
                MathUtil.isNear(
                    position.getAsDouble(), inputs.positionUnits, acceptableError.getAsDouble()))
        .withName("SmartPositionControlUntilOnTarget");
  }

  public Command smartPositionSetpointUntilOnTargetCommand(
      DoubleSupplier position, DoubleSupplier acceptableError) {
    return smartPositionSetpointUntilOnTargetCommand(position, acceptableError, 0);
  }

  // --- COMMANDS: Dynamic Smart Position Control

  public Command dynamicSmartPositionSetpointCommand(
      DoubleSupplier position, Supplier<MotionMagicConfigs> config, int slot) {
    return runEnd(
            () -> {
              setDynamicSmartPositionSetpointImpl(position.getAsDouble(), config.get(), slot);
            },
            () -> {})
        .withName("DynamicSmartPositionControl");
  }

  public Command dynamicSmartPositionSetpointCommand(
      DoubleSupplier position, Supplier<MotionMagicConfigs> config) {
    return dynamicSmartPositionSetpointCommand(position, config, 0);
  }

  public Command dynamicSmartPositionSetpointUntilOnTargetCommand(
      DoubleSupplier position,
      DoubleSupplier acceptableError,
      Supplier<MotionMagicConfigs> config,
      int slot) {
    return dynamicSmartPositionSetpointCommand(position, config, slot)
        .until(
            () ->
                MathUtil.isNear(
                    position.getAsDouble(), inputs.positionUnits, acceptableError.getAsDouble()))
        .withName("DynamicSmartPositionControlUntilOnTarget");
  }

  public Command dynamicSmartPositionSetpointUntilOnTargetCommand(
      DoubleSupplier position,
      DoubleSupplier acceptableError,
      Supplier<MotionMagicConfigs> config) {
    return dynamicSmartPositionSetpointUntilOnTargetCommand(position, acceptableError, config, 0);
  }
}
