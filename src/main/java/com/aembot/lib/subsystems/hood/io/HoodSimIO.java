package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.simulation.SimulatedHoodConfiguration;
import com.aembot.lib.core.motors.SimulatedTalonFX;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.hood.HoodInputs;

public class HoodSimIO implements HoodIO {

  private final SimulatedTalonFX simMotor;

  public final SimulatedHoodConfiguration config;

  public HoodSimIO(SimulatedHoodConfiguration config) {
    this.config = config;
    this.simMotor = new SimulatedTalonFX(config.kSimMotorConfig);
  }

  @Override
  public MotorIOTalonFXSim getMotor() {
    return simMotor.getMotor();
  }

  @Override
  public void updateInputs(HoodInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.updateLog(standardPrefix, inputPrefix);
  }
}
