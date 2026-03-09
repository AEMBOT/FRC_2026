package com.aembot.frc2026.subsystems.indexerSelector.io;

import com.aembot.frc2026.config.subsystems.indexerSelector.IndexerSelectorConfiguration;
import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorMechanismInputs;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.tracing.Traced;
import edu.wpi.first.wpilibj.Notifier;

public class IndexerSelectorMechanismIOSim implements IndexerSelectorMechanismIO {
  /** Simulated motor IO */
  private final MotorIOTalonFXSim kSimMotor;

  /** Configuration for this simulation */
  public final IndexerSelectorConfiguration kConfig;

  /** Notifier for this simulation */
  private final Notifier kSimNotifier;

  /**
   * @param config Configuration for this simulation
   */
  public IndexerSelectorMechanismIOSim(IndexerSelectorConfiguration config) {
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
  @Traced
  public void updateInputs(IndexerSelectorMechanismInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {
    kSimMotor.logSim(standardPrefix, inputPrefix);
  }
}
