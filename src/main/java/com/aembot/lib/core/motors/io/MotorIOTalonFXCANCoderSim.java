package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class MotorIOTalonFXCANCoderSim extends MotorIOTalonFXSim {
  private final CANcoderSimState cancoderSimState;

  /**
   * Create a new simulated Talon FX paired with a CANcoder
   *
   * @param servoMotorConfig The {@link MotorConfiguration}, for... the motor to be configurationed
   * @param encoderDevice CAN Device representing the encoder
   * @param cancoder CANcoder object itself that we are able to yoink the sim state from
   */
  public MotorIOTalonFXCANCoderSim(
      MotorConfiguration<TalonFXConfiguration> servoMotorConfig,
      CANDeviceID encoderDevice,
      CANcoder cancoder) {
    super(servoMotorConfig);

    this.cancoderSimState = cancoder.getSimState();
  }

  /**
   * Create a new simulated Talon FX paired with a CANcoder
   *
   * @param motorDevice CAN Device representing the motor
   * @param motorConfig TalonFXConfiguration defining just the functionality of the motor
   * @param cancoder CANcoder object itself that we are able to yoink the sim state from
   */
  public MotorIOTalonFXCANCoderSim(
      CANDeviceID motorDevice, TalonFXConfiguration motorConfig, CANcoder cancoder) {
    super(motorDevice, motorConfig);

    this.cancoderSimState = cancoder.getSimState();
  }

  /**
   * Create a new simulated Talon FX paired with a CANcoder
   *
   * @param motorDevice CAN Device representing the motor
   * @param motor TalonFX object itself to use as the motor
   * @param cancoder CANcoder object itself that we are able to rip the sim state from
   */
  public MotorIOTalonFXCANCoderSim(TalonFX motor, CANcoder cancoder) {
    super(motor);

    this.cancoderSimState = cancoder.getSimState();
  }

  /**
   * Forcibly update the simulated control signals for both the CAN coder simulation and the motor
   * simulation
   *
   * @param mechanismAngle The current angular position of the mechanism (e.g., arm, wheel) that the
   *     motor is driving. Given to cancoder as its position.
   * @param mechanismVelocity The current angular velocity of the mechanism. Given to the cancoder
   *     as its velocity.
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
    cancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    cancoderSimState.setRawPosition(mechanismAngle);
    cancoderSimState.setVelocity(mechanismVelocity);

    return super.updateControlSignal(
        mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
  }
}
