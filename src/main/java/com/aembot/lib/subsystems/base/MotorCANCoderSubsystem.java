package com.aembot.lib.subsystems.base;

import com.aembot.lib.config.motors.MotorCANCoderConfiguration;
import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;

/**
 * Flexible base subsystem for any {@link MotorIO}-driven mechanism that can behave like a servo,
 * paired with a CANCoder for zeroing (or fused telemetry). It wraps a configurable motor
 * controller, handles telemetry, and exposes commands for open-loop and closed-loop
 * position/velocity control.
 *
 * <p>Key capabilities provided by this class include:
 *
 * <ul>
 *   <li>Polling {@link MotorInputs} and pushing telemetry to {@link
 *       org.littletonrobotics.junction.Logger AKit's Logger}
 *   <li>Helper methods for managing encoder offsets and setpoints
 *   <li>Automatically zeroing the motor's encoder to the absolute encoder if not using fused
 *   <li>Command factories for controlling with Motion Magic, PID, & duty-cycle commands
 * </ul>
 *
 * @param <MI> {@link MotorInputs} class capturing feedback for this motor.
 * @param <M> {@link MotorIO} IO interface for the motor.
 * @param <EI> {@link CANCoderInputs} class capturing feedback for the CANCoder.
 * @param <E> {@link CANCoderIO} IO interface for the CANCoder.
 * @param <C> {@link MotorConfiguration} describing behaviour of the motor
 */
public abstract class MotorCANCoderSubsystem<
        MI extends MotorInputs,
        M extends MotorIO,
        EI extends CANCoderInputs,
        E extends CANCoderIO,
        C extends MotorCANCoderConfiguration<?>>
    extends MotorSubsystem<MI, M, C> {

  // Inputs to be used for this CANcoder
  protected EI canCoderInputs;

  // The instance of CANCoder IO itself
  protected E canCoder;

  // Has the "motors" position been offset yet to use the CANcoders position instead of the internal
  // motor position so that we always know where the motor is on initialization
  protected boolean hasSetOffset = false;

  /** Creates a new ServoMotorSubsystemCanCoder. */
  public MotorCANCoderSubsystem(
      MI motorInputs, M motor, EI canCoderInputs, E canCoder, C motorConfiguration) {
    super(
        "Subsystems/" + motorConfiguration.configurationName + "/" + canCoder.getName(),
        motorInputs,
        motor,
        motorConfiguration);

    this.canCoderInputs = canCoderInputs;
    this.canCoder = canCoder;
  }

  @Override
  public void periodic() {
    super.periodic();

    canCoder.updateInputs(canCoderInputs);
    Logger.processInputs(logPrefixInput + "/Inputs", canCoderInputs);

    // If this encoder is not fused, has a valid location and the offset hasn't already been set,
    // update the motor's encoder's value to the absolute encoder's value
    if (!this.motorConfig.isFusedCANCoder
        && !this.hasSetOffset
        && !Double.isNaN(canCoderInputs.absolutePositionRotations)) {
      updateOffsetImpl();
      this.hasSetOffset = true;
    }
  }

  /**
   * Implementation to update what the motor thinks its encoder position is to match what the
   * CANcoder is telling it (since absolute it gives us a really solid zero)
   */
  protected void updateOffsetImpl() {
    io.setCurrentEncoderPosition(
        this.motorConfig.getEncoderRotationsToUnits(canCoderInputs.absolutePositionRotations));
  }

  /** Command factory to run to rezero the motor's relative encoder to the absolute encoder */
  public Command updateCANCoderOffsetCommand() {
    return new InstantCommand(() -> updateOffsetImpl());
  }
}
