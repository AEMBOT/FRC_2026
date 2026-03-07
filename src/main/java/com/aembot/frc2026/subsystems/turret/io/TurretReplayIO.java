package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.core.tracing.Traced;

/** Replay implementation for turret IO */
public class TurretReplayIO implements TurretIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  public CANCoderIO getCANcoderA() {
    return null;
  }

  @Override
  public CANCoderIO getCANcoderB() {
    return null;
  }

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  @Traced
  public void updateInputs(TurretInputs inputs) {}
}
