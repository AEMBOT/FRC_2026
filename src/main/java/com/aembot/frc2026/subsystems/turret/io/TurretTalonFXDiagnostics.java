package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.phoenix6.CTREUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.wpilibj.Timer;

/** Refreshes turret-only profile signals alongside the standard motor IO feedback path. */
final class TurretTalonFXDiagnostics {
  private final MotorConfiguration<?> kConfig;
  private final StatusSignal<Double> kReference;
  private final StatusSignal<Double> kReferenceSlope;
  private final StatusSignal<Double> kError;
  private final StatusSignal<ControlModeValue> kControlMode;
  private final BaseStatusSignal kPosition;
  private final BaseStatusSignal kVelocity;
  private final BaseStatusSignal kVoltage;
  private final BaseStatusSignal kStatorCurrent;
  private final BaseStatusSignal kSupplyCurrent;
  private final BaseStatusSignal[] kExtraSignals;
  private final BaseStatusSignal[] kAllSignals;

  TurretTalonFXDiagnostics(TalonFX motor, MotorConfiguration<?> config) {
    kConfig = config;
    kReference = motor.getClosedLoopReference(false);
    kReferenceSlope = motor.getClosedLoopReferenceSlope(false);
    kError = motor.getClosedLoopError(false);
    kControlMode = motor.getControlMode(false);
    // These are the same cached signals MotorIOTalonFX already refreshes.
    kPosition = motor.getPosition(false);
    kVelocity = motor.getVelocity(false);
    kVoltage = motor.getMotorVoltage(false);
    kStatorCurrent = motor.getStatorCurrent(false);
    kSupplyCurrent = motor.getSupplyCurrent(false);
    kExtraSignals = new BaseStatusSignal[] {kReference, kReferenceSlope, kError, kControlMode};
    kAllSignals =
        new BaseStatusSignal[] {
          kReference,
          kReferenceSlope,
          kError,
          kControlMode,
          kPosition,
          kVelocity,
          kVoltage,
          kStatorCurrent,
          kSupplyCurrent
        };
    // Reference signals default to only 4 Hz on CAN 2.0. Match the 50 Hz motor feedback.
    CTREUtil.setUpdateFrequencyForAll(50.0, kExtraSignals, motor.getDeviceID());
  }

  void updateInputs(TurretInputs inputs) {
    // Like the standard motor IO, use a zero-timeout refresh on the main robot thread.
    BaseStatusSignal.refreshAll(kExtraSignals);
    inputs.available = true;
    inputs.readTimestampSeconds = Timer.getFPGATimestamp();
    double now = Utils.getCurrentTimeSeconds();
    inputs.signalsOK = BaseStatusSignal.isAllGood(kAllSignals);
    inputs.timestampsValid = true;
    inputs.oldestSignalAgeSeconds = 0.0;
    for (BaseStatusSignal signal : kAllSignals) {
      inputs.timestampsValid &= signal.getTimestamp().isValid();
      inputs.oldestSignalAgeSeconds = Math.max(inputs.oldestSignalAgeSeconds, age(signal, now));
    }
    inputs.controlMode = kControlMode.getValue().toString();
    // These conversions are meaningful in position modes; keep ControlMode next to them in logs.
    inputs.referenceDegrees = kConfig.getRotorRotationsToUnits(kReference.getValueAsDouble());
    inputs.referenceVelocityDegreesPerSecond =
        kConfig.getRotorRotationsToUnits(kReferenceSlope.getValueAsDouble());
    inputs.controllerErrorDegrees = kConfig.getRotorRotationsToUnits(kError.getValueAsDouble());
    inputs.referenceAgeSeconds = age(kReference, now);
    inputs.referenceVelocityAgeSeconds = age(kReferenceSlope, now);
    inputs.positionAgeSeconds = age(kPosition, now);
    inputs.velocityAgeSeconds = age(kVelocity, now);
    inputs.outputSignalsAgeSeconds =
        Math.max(age(kVoltage, now), Math.max(age(kStatorCurrent, now), age(kSupplyCurrent, now)));
  }

  private static double age(BaseStatusSignal signal, double now) {
    return signal.getTimestamp().isValid() ? now - signal.getTimestamp().getTime() : Double.NaN;
  }
}
