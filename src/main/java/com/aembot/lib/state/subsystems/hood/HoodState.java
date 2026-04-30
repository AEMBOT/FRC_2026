package com.aembot.lib.state.subsystems.hood;

import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.concurrent.atomic.AtomicReference;

public class HoodState implements Loggable {
  public AtomicReference<Rotation2d> hoodAngle = new AtomicReference<>();

  public Rotation2d getHoodAngle() {
    return hoodAngle.get();
  }

  public void updateHoodAngle(Rotation2d value) {
    hoodAngle.set(value);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    AEMLogger.recordOutput(standardPrefix + "/Angle", hoodAngle.get());
  }
}
