package com.aembot.lib.state.subsystems.flywheel;

import com.aembot.lib.core.logging.Loggable;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class FlywheelState implements Loggable {
  public AtomicReference<Double> flywheelSpeedUnitsPerSecond = new AtomicReference<>();
  public AtomicBoolean atAcceptableSpeed = new AtomicBoolean();

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/FlywheelSpeedMPS", flywheelSpeedUnitsPerSecond.get());
    Logger.recordOutput(standardPrefix + "/AtAcceptableSpeed", atAcceptableSpeed.get());
  }
}
