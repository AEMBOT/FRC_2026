package com.aembot.frc2026.subsystems.turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Inputs for turret-specific telemetry that doesn't belong in MotorInputs */
public class TurretInputs implements LoggableInputs {
  public double encoderAAbsolutePositionRotations = 0.0;
  public double encoderAVelocityRotationsPerSecond = 0.0;
  public double encoderBAbsolutePositionRotations = 0.0;
  public double encoderBVelocityRotationsPerSecond = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("EncoderAAbsolutePositionRotations", encoderAAbsolutePositionRotations);
    table.put("EncoderAVelocityRotationsPerSecond", encoderAVelocityRotationsPerSecond);
    table.put("EncoderBAbsolutePositionRotations", encoderBAbsolutePositionRotations);
    table.put("EncoderBVelocityRotationsPerSecond", encoderBVelocityRotationsPerSecond);
  }

  @Override
  public void fromLog(LogTable table) {
    encoderAAbsolutePositionRotations =
        table.get("EncoderAAbsolutePositionRotations", encoderAAbsolutePositionRotations);
    encoderAVelocityRotationsPerSecond =
        table.get("EncoderAVelocityRotationsPerSecond", encoderAVelocityRotationsPerSecond);
    encoderBAbsolutePositionRotations =
        table.get("EncoderBAbsolutePositionRotations", encoderBAbsolutePositionRotations);
    encoderBVelocityRotationsPerSecond =
        table.get("EncoderBVelocityRotationsPerSecond", encoderBVelocityRotationsPerSecond);
  }
}
