package com.aembot.frc2026.state.subsystems.turret;

import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.concurrent.atomic.AtomicReference;

public class TurretState implements Loggable {
  public AtomicReference<Rotation2d> turretYaw = new AtomicReference<>();

  public void updateTurretYaw(Rotation2d yaw) {
    turretYaw.set(yaw);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    AEMLogger.recordOutput(standardPrefix + "/TurretYaw", turretYaw.get());
  }
}
