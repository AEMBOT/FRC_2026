package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** IO implementation for a simulated TalonFX */
public class MotorIOTalonFXSim extends MotorIOTalonFX implements SimulatedMotorController {
  protected final TalonFXSimState simState;

  /**
   * Create a new TalonFX Sim IO using a ServoMotorConfiguration.
   *
   * @param device CAN Device representing the Talon Fx
   * @param servoMotorConfig The servo motor configuration to create the new TalonFXIO with
   */
  public MotorIOTalonFXSim(MotorConfiguration<TalonFXConfiguration> servoMotorConfig) {
    super(servoMotorConfig);

    simState = talon.getSimState();
    simState.Orientation =
        MotorIOTalonFXSim.computeSimMotorOrientation(
            servoMotorConfig.getMotorConfig().MotorOutput.Inverted);
  }

  /**
   * Create a new TalonFX Sim IO using a TalonFXConfiguration.
   *
   * @param device CAN Device representing the Talon Fx
   * @param motorConfig The basic motor configuration for this TalonFX
   */
  public MotorIOTalonFXSim(CANDeviceID device, TalonFXConfiguration motorConfig) {
    super(device, motorConfig);

    simState = talon.getSimState();
    simState.Orientation =
        MotorIOTalonFXSim.computeSimMotorOrientation(motorConfig.MotorOutput.Inverted);
  }

  /**
   * Create a new TalonFX Sim IO using a raw motor, this should pretty much ONLY be used on the
   * drivetrain
   *
   * @param motor The motor itself, the TalonFXIO is just a commonality wrapper
   */
  public MotorIOTalonFXSim(TalonFX motor) {
    super(motor);
    simState = talon.getSimState();
  }

  public TalonFXSimState getSimState() {
    return simState;
  }

  /**
   * Forcibly updates the internal simulation state of the motor using measured encoder values and
   * battery voltage, then calculates and returns the corresponding motor voltage measure.
   *
   * <p>This method is intended for use within a simulation environment to synchronize the simulated
   * motor model's state (position, velocity, supply voltage) with the external measurements being
   * fed to it (e.g., from an encoder or another physics model).
   *
   * @param mechanismAngle The current angular position of the mechanism (e.g., arm, wheel) that the
   *     motor is driving. Not directly used in the current implementation.
   * @param mechanismVelocity The current angular velocity of the mechanism. Not directly used in
   *     the current implementation.
   * @param encoderAngle The latest angular position reading from the motor's encoder. This is used
   *     to set the motor's raw rotor position in the simulation.
   * @param encoderVelocity The latest angular velocity reading from the motor's encoder. This is
   *     used to set the motor's rotor velocity in the simulation.
   * @return A {@link Voltage} object representing the calculated motor voltage measure that should
   *     be applied to the motor for the next simulation step, based on the updated state.
   */
  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    simState.setRawRotorPosition(encoderAngle);
    simState.setRotorVelocity(encoderVelocity);
    simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

    return simState.getMotorVoltageMeasure();
  }

  /**
   * Compute the proper chassis orientation given some motor inverted value
   *
   * @param value The inverted direction of the motor
   * @return The chassis centric orientation of the motor
   */
  private static ChassisReference computeSimMotorOrientation(InvertedValue value) {
    return (value == InvertedValue.Clockwise_Positive)
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }
}
