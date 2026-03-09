package com.aembot.frc2026.subsystems.spindexer.io;

import com.aembot.frc2026.config.subsystems.spindexer.SpindexerConfiguration;
import com.aembot.frc2026.subsystems.spindexer.SpindexerMechanismInputs;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.tracing.Traced;
import edu.wpi.first.wpilibj.Notifier;

public class SpindexerMechanismIOSim implements SpindexerMechanismIO {
  /** Simulated motor IO */
  private final MotorIOTalonFXSim kSimMotor;

  /** Configuration for this simulation */
  public final SpindexerConfiguration kConfig;

  /** Notifier for this simulation */
  private final Notifier kSimNotifier;

  /**
   * @param config Configuration for this simulation
   */
  public SpindexerMechanismIOSim(SpindexerConfiguration config) {
    this.kConfig = config;
    this.kSimMotor = new MotorIOTalonFXSim(config.kSimMotorConfig);
    this.kSimNotifier = new Notifier(() -> kSimMotor.updateSimState());
    kSimNotifier.setName(config.kMotorConfig.kConfigurationName + "Notifier");
    kSimNotifier.startPeriodic(0.005);
  }

  @Override
  public MotorIOTalonFXSim getMotor() {
    return kSimMotor;
  }

  @Override
  @Traced(category = "Spindexer")
  public void updateInputs(SpindexerMechanismInputs inputs) {}

  @Override
  @Traced(category = "Spindexer")
  public void updateLog(String standardPrefix, String inputPrefix) {
    kSimMotor.logSim(standardPrefix, inputPrefix);
  }
}
