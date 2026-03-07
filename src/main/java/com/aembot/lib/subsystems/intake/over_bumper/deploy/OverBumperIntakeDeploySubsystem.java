package com.aembot.lib.subsystems.intake.over_bumper.deploy;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.state.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.OverBumperIntakeDeployIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Extension of the motor subsystem to add over the bumper intake deployment functionality */
public class OverBumperIntakeDeploySubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final OverBumperIntakeDeployIO io;

  /** Configuration to use for this subsystem */
  private final TalonFXOverBumperIntakeDeployConfiguration config;

  /** State of this subsystem */
  private final OverBumperIntakeDeployState state;

  /** consumer in order to update values in robot state */
  private final Consumer<OverBumperIntakeDeployState> stateConsumer;

  /**
   * Construct a new over the bumper intake deployment subsystem
   *
   * @param config Configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   * @param stateConsumer State consumer in order to update the state of this subsystem in
   *     RobotState
   */
  public OverBumperIntakeDeploySubsystem(
      TalonFXOverBumperIntakeDeployConfiguration config,
      OverBumperIntakeDeployIO io,
      Consumer<OverBumperIntakeDeployState> stateConsumer) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kRealMotorConfig);
    this.io = io;
    this.config = config;
    this.state = new OverBumperIntakeDeployState();
    this.stateConsumer = stateConsumer;

    this.setEncoderPosition(config.kInitialAngleDeg);
  }

  /**
   * @return A command that runs the motor upwards until it reaches a hard stop, then resets encoder
   *     position
   */
  public Command getZeroUpwardCommand() {
    Timer timer = new Timer();
    return new InstantCommand(
            () -> {
              setEncoderPosition(config.kRealMotorConfig.kMinPositionUnits);
              timer.restart();
            })
        .andThen(
            voltageCommand(() -> config.kZeroingVoltage)
                .until(
                    () -> (MathUtil.isNear(0, getCurrentVelocity(), 0.5) && timer.hasElapsed(0.1)))
                .finallyDo(
                    () -> {
                      setEncoderPosition(
                          config.kRealMotorConfig.getUnitsToRotorRotations(
                              config.kUpwardsZeroAngleDeg));
                    }));
  }

  /**
   * @return A command that runs the motor downwards until it reaches a hard stop, then resets
   *     encoder position
   */
  public Command getZeroDownwardCommand() {
    Timer timer = new Timer();
    return new InstantCommand(
            () -> {
              setEncoderPosition(config.kRealMotorConfig.kMaxPositionUnits);
              timer.restart();
            })
        .andThen(
            voltageCommand(() -> -config.kZeroingVoltage)
                .until(
                    () -> (MathUtil.isNear(0, getCurrentVelocity(), 0.5) && timer.hasElapsed(0.5)))
                .finallyDo(
                    () -> {
                      setEncoderPosition(
                          config.kRealMotorConfig.getUnitsToRotorRotations(
                              config.kDownwardsZeroAngleDeg));
                    }));
  }

  /**
   * @return A command that puts the motor to the up position
   */
  public Command putIntakeUpCommand() {
    return smartPositionSetpointCommand(() -> config.kRealMotorConfig.kMaxPositionUnits);
  }

  /**
   * @return A command that puts the motor to the down position
   */
  public Command putIntakeDownCommand() {
    return smartPositionSetpointCommand(() -> config.kRealMotorConfig.kMinPositionUnits);
  }

  public Command flickIntakeCommand() {
    return putIntakeUpCommand()
        .withTimeout(0.35)
        .andThen(
            putIntakeDownCommand()
                .until(() -> MathUtil.isNear(getCurrentPosition(), getPositionSetpointUnits(), 2)));
  }

  @Traced
  private void updateState() {

    state.deployPositionUnits = getCurrentPosition();

    state.isDeployed =
        state.deployPositionUnits
            < (config.kRealMotorConfig.kMaxPositionUnits
                    + config.kRealMotorConfig.kMinPositionUnits)
                / 2;

    stateConsumer.accept(state);
  }

  @Override
  @Traced
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    super.periodic();

    updateState();

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    state.updateLog(standardPrefix + "/state", inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
