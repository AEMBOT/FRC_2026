package com.aembot.lib.subsystems.premades;

import com.aembot.lib.config.subsystems.intake.generic.run.BinaryVoltageMotorFollowerConfig;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.containers.CompoundMotorIO;
import com.aembot.lib.subsystems.base.MotorFollowerSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

/**
 * Extension of the motor follower subsystem to add over the bumper intake roller functionality.
 * Good for Texas Toast.
 */
public class BinaryVoltageMotorFollowerSubsytem
    extends MotorFollowerSubsystem<MotorInputs, MotorIO, TalonFXConfiguration> {

  private final BinaryVoltageMotorFollowerConfig kConfig;

  /**
   * Construct a new over the bumper intake roller subsystem
   *
   * @param config configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   * @param state State consumer in order to update the state of this subsystem in RobotState
   */
  public BinaryVoltageMotorFollowerSubsytem(
      BinaryVoltageMotorFollowerConfig config, CompoundMotorIO<MotorIO> motorIOContainer) {
    // super(
    //     new MotorInputs(),
    //     motors[0],
    //     Stream.generate(MotorInputs::new)
    //         .limit(config.validate().kMotorConfigs.followerConfigurations.size())
    //         .toArray(MotorInputs[]::new),
    //     Arrays.copyOfRange(motors, 1, motors.length),
    //     config.kMotorConfigs);
    super(
        Stream.generate(MotorInputs::new)
            .limit(motorIOContainer.kMotors.size())
            .toArray(MotorInputs[]::new),
        motorIOContainer,
        config.kMotorConfigs);

    this.kConfig = config;
  }

  public Command runSystemCommand() {
    return voltageCommand(() -> kConfig.kRunVoltage);
  }

  public Command reverseSystemCommand() {
    return voltageCommand(() -> -kConfig.kRunVoltage);
  }

  public Command stopSystemCommand() {
    return voltageCommand(() -> 0);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    super.periodic();

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);
  }
}
