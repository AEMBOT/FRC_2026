package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.simulation.SimulatedHoodConfiguration;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.hood.HoodInputs;
import edu.wpi.first.wpilibj.Notifier;

public class HoodSimIO implements HoodIO {

  private final MotorIOTalonFXSim simMotor;

  public final SimulatedHoodConfiguration config;

  private final Notifier simNotifier;

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
  public void updateInputs(HoodInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
  }
}
