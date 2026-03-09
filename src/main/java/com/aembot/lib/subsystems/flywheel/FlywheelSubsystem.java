package com.aembot.lib.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.state.subsystems.flywheel.FlywheelState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.flywheel.io.FlywheelIO;
import com.aembot.lib.tracing.Traced;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private final FlywheelIO flywheel;
  private final TalonFXFlywheelConfiguration config;

  /** Flywheel state instance to update */
  private final FlywheelState state;

  public FlywheelSubsystem(
      TalonFXFlywheelConfiguration config, FlywheelIO flywheel, FlywheelState stateInstance) {
    super(new MotorInputs(), flywheel.getMotor(), config.kMotorConfig);

    this.flywheel = flywheel;
    this.config = config;
    this.state = stateInstance;

    zeroEncoderPosition();
    SmartDashboard.putBoolean("Flywheel Enabled", motorEnabled);
  }

  @Override
  @Traced(category = "Flywheel")
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    super.periodic();

    state.flywheelSpeedUnitsPerSecond.set(inputs.velocityUnitsPerSecond);
    state.atAcceptableSpeed.set(
        Math.abs(inputs.velocityUnitsPerSecond - this.currentVelocitySetpoint)
            < config.kSpeedToleranceUnitsPerSecond);

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    motorEnabled = SmartDashboard.getBoolean("Flywheel Enabled", true);
  }

  @Override
  @Traced(category = "Flywheel")
  public void updateLog(String standardPrefix, String inputPrefix) {
    flywheel.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
