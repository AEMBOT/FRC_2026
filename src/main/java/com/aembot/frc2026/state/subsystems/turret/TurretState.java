package com.aembot.frc2026.state.subsystems.turret;

import com.aembot.lib.core.logging.Loggable;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class TurretState implements Loggable {
  public AtomicReference<Double> turretYawRadians = new AtomicReference<>();

  public void updateTurretYaw(Double yaw) {
    turretYawRadians.set(yaw);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/TurretYaw", turretYawRadians.get());
  }
}
