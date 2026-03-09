package com.aembot.lib.subsystems.flywheel.io;

import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.core.motors.factories.TalonFXFactory;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.flywheel.FlywheelInputs;
import com.aembot.lib.tracing.Traced;

/** Hardware IO implementation for a flywheel */
public class FlywheelHardwareIO implements FlywheelIO {
  private final MotorIOTalonFX flywheelMotor;

  public FlywheelHardwareIO(TalonFXFlywheelConfiguration flywheel) {
    // Setup our leader motor based on the configuration
    flywheelMotor = TalonFXFactory.createIO(flywheel.kMotorConfig);
  }

  @Override
  @Traced(category = "Flywheel")
  public void updateInputs(FlywheelInputs inputs) {
    updateLog();
  }

  @Override
  @Traced(category = "Flywheel")
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public MotorIO getMotor() {
    return flywheelMotor;
  }
}
