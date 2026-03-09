package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.simulation.SimulatedHoodConfiguration;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.hood.HoodInputs;
import com.aembot.lib.tracing.Traced;
import edu.wpi.first.wpilibj.Notifier;

/** Simulated Hood IO */
public class HoodSimIO implements HoodIO {

  /** Simulated motor IO */
  private final MotorIOTalonFXSim simMotor;

  /** Configuration for this simulation */
  public final SimulatedHoodConfiguration config;

  /** Notifier for this simulation */
  private final Notifier simNotifier;

  /**
   * Construct a new Simulated hood IO
   *
   * @param config Configuration for this simulation
   */
  public HoodSimIO(SimulatedHoodConfiguration config) {
    this.config = config;
    this.simMotor = new MotorIOTalonFXSim(config.kSimMotorConfig);
    this.simNotifier = new Notifier(() -> simMotor.updateSimState());
    simNotifier.setName(config.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public MotorIOTalonFXSim getMotor() {
    return simMotor;
  }

  @Override
  @Traced(category = "Hood")
  public void updateInputs(HoodInputs inputs) {}

  @Override
  @Traced(category = "Hood")
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
  }
}
