package com.aembot.lib.state.subsystems.hood;

import com.aembot.lib.core.logging.Loggable;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class HoodState implements Loggable {
  public AtomicReference<Double> hoodAngleRadians = new AtomicReference<>();

  public Double getHoodAngleRadians() {
    return hoodAngleRadians.get();
  }

  public void updateHoodAngle(Double valueRadians) {
    hoodAngleRadians.set(valueRadians);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/AngleRads", hoodAngleRadians.get());
  }
}
