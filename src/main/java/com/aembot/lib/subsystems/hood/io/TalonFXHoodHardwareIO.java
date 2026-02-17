package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.hood.HoodInputs;

/** Real Hood IO */
public class TalonFXHoodHardwareIO implements HoodIO {

  /** Motor IO to use */
  private final MotorIOTalonFX motor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXHoodConfiguration config;

  /**
   * Construct a new Real Hood IO
   *
   * @param config Configuration to use for this io
   */
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
