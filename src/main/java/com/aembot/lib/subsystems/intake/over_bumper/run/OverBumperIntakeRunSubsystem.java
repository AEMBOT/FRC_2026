package com.aembot.lib.subsystems.intake.over_bumper.run;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.intake.over_bumper.run.TalonFXOverBumperIntakeRunConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRunState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.io.OverBumperIntakeRunIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * Extension of the motor subsystem to add over the bumper intake game piece intaking functionality
 */
public class OverBumperIntakeRunSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final OverBumperIntakeRunIO io;

  private final OverBumperIntakeRunState state;

  /** Configuration to use for this subsystem */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXOverBumperIntakeRunConfiguration config;

  private final Consumer<OverBumperIntakeRunState> stateConsumer;

  /**
   * Construct a new over the bumper intake game piece intaking subsystem
   *
   * @param config configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   */
  public OverBumperIntakeRunSubsystem(
      TalonFXOverBumperIntakeRunConfiguration config,
      OverBumperIntakeRunIO io,
      Consumer<OverBumperIntakeRunState> stateConsumer) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.io = io;
    this.config = config;
    this.state = new OverBumperIntakeRunState();
    this.stateConsumer = stateConsumer;
  }

  private void updateState() {

    state.angularVelocityUnitsPerMin = getCurrentVelocity();

    state.isActive =
        state.angularVelocityUnitsPerMin > config.kMotorConfig.getMechanismRotationsToUnits(1);

    stateConsumer.accept(state);
  }

  @Override
  public void periodic() {
    super.periodic();
    updateState();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    state.updateLog(standardPrefix + "/state", inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
