package com.aembot.lib.subsystems.flywheel.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.subsystems.flywheel.FlywheelInputs;

public class FlywheelReplayIO implements FlywheelIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  @Traced
  public void updateInputs(FlywheelInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
