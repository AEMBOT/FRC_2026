package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.hood.HoodInputs;

public class TalonFXHoodHardwareIO implements HoodIO {

  private final MotorIOTalonFX motor;

  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXHoodConfiguration config;

  public TalonFXHoodHardwareIO(TalonFXHoodConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kMotorConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    updateLog();
  }

  @Override
  public void updateLog(String standardPrefix, String inputs) {}
}
