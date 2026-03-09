package com.aembot.lib.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.state.subsystems.hood.HoodState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIO;
import com.aembot.lib.tracing.Traced;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/** Extension of the motor subsystem to add hood functionality */
public class HoodSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final HoodIO io;

  /** Configuration to use for this subsystem */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXHoodConfiguration config;

  /** The HoodState instance to update */
  private final HoodState state;

  /**
   * Construct a new hood subsystem
   *
   * @param config Configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   */
  public HoodSubsystem(TalonFXHoodConfiguration config, HoodIO io, HoodState state) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.io = io;
    this.config = config;
    this.state = state;

    this.setEncoderPosition(config.upwardsHardStopUnits);

    SmartDashboard.putBoolean("Hood Enabled", motorEnabled);
  }

  @Override
  @Traced(category = "Hood")
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    super.periodic();

    state.updateHoodAngle(new Rotation2d(Units.degreesToRadians(inputs.positionUnits)));

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    motorEnabled = SmartDashboard.getBoolean("Hood Enabled", true);
  }

  @Override
  @Traced(category = "Hood")
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
