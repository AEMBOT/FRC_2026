package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.simulation.SimulatedHoodConfiguration;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.hood.HoodInputs;

public class HoodSimIO implements HoodIO {

  public final MotorIOTalonFXSim motor;

  public final SimulatedHoodConfiguration config;

  public HoodSimIO(SimulatedHoodConfiguration config) {
    this.config = config;
    motor = new MotorIOTalonFXSim(config.kMotorConfig);
  }

  @Override
  public MotorIOTalonFXSim getMotor() {
    return motor;
  }

  @Override
  public void updateInputs(HoodInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
