package com.aembot.lib.subsystems.intake.generic.multimotor;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.subsystems.intake.generic.run.MultiTalonFXIntakeRollerConfig;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.state.subsystems.intake.generic.run.IntakeRollerState;
import com.aembot.lib.subsystems.base.MotorFollowerSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Consumer;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

/** Extension of the motor follower subsystem to add over the bumper intake roller functionality. Good for Texas Toast. */
public class IntakeRollerMultiMotorSubsystem
    extends MotorFollowerSubsystem<
        MotorInputs, MotorIO, MotorFollowersConfiguration<TalonFXConfiguration>> {

  private final MultiTalonFXIntakeRollerConfig kConfig;
  private final IntakeRollerState kState;

  /**
   * Construct a new over the bumper intake roller subsystem
   *
   * @param config configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   * @param state State consumer in order to update the state of this subsystem in
   *     RobotState
   */
  public IntakeRollerMultiMotorSubsystem(
      MultiTalonFXIntakeRollerConfig config,
      IntakeRollerState state,
      MotorIO... motors) {
    super(
        new MotorInputs(),
        motors[0],
        Stream.generate(MotorInputs::new)
            .limit(config.validate().kMotorConfigs.followerConfigurations.size())
            .toArray(MotorInputs[]::new),
        motors,
        config.kMotorConfigs);

    this.kConfig = config;
    this.kState = state;
  }

  public Command runRollerCommand() {
    return voltageCommand(() -> kConfig.kIntakeVoltage);
  }

  public Command stopRollerCommand() {
    return voltageCommand(() -> 0);
  }

  private void updateState() {
    kState.angularVelocityUnitsPerMin.set(getCurrentVelocity());
    kState.isActive.set(
        kState.angularVelocityUnitsPerMin.get() > kConfig.kMotorConfigs.getMechanismRotationsToUnits(1));
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    super.periodic();
    updateState();

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    kState.updateLog(standardPrefix + "/state", inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
