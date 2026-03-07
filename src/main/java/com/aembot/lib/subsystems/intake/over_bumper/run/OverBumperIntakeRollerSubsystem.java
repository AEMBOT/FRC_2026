package com.aembot.lib.subsystems.intake.over_bumper.run;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.run.TalonFXOverBumperIntakeRollerConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRollerState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.io.OverBumperIntakeRollerIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Extension of the motor subsystem to add over the bumper intake roller functionality */
public class OverBumperIntakeRollerSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final OverBumperIntakeRollerIO io;

  /** State of this subsystem */
  private final OverBumperIntakeRollerState state;

  /** Configuration to use for this subsystem */
  private final TalonFXOverBumperIntakeRollerConfiguration config;

  /** consumer in order to update values in robot state */
  private final Consumer<OverBumperIntakeRollerState> stateConsumer;

  /**
   * Construct a new over the bumper intake roller subsystem
   *
   * @param config configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   * @param stateConsumer State consumer in order to update the state of this subsystem in
   *     RobotState
   */
  public OverBumperIntakeRollerSubsystem(
      TalonFXOverBumperIntakeRollerConfiguration config,
      OverBumperIntakeRollerIO io,
      Consumer<OverBumperIntakeRollerState> stateConsumer) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kRealMotorConfig);
    this.io = io;
    this.config = config;
    this.state = new OverBumperIntakeRollerState();
    this.stateConsumer = stateConsumer;
  }

  public Command runRollerCommand() {
    return voltageCommand(() -> config.kIntakeVoltage);
  }

  public Command stopRollerCommand() {
    return voltageCommand(() -> 0);
  }

  @Traced
  private void updateState() {

    state.angularVelocityUnitsPerMin = getCurrentVelocity();

    state.isActive =
        state.angularVelocityUnitsPerMin > config.kRealMotorConfig.getMechanismRotationsToUnits(1);

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
