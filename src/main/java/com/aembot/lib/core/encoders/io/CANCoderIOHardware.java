package com.aembot.lib.core.encoders.io;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.interfaces.CANable;
import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.encoders.factories.CANCoderFactory;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CANCoderIOHardware implements CANCoderIO, CANable {
  protected final CANcoder encoder;

  protected AEMCANCoderConfiguration config;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  /** All the status signals we are using for this CANCoder */
  private final BaseStatusSignal[] signals;

  /** The number of cycles we've spent warming up CAN readings. See updateInputs() */
  private int warmupReadingCount = 0;

  /** The number of cycles to spent warming up CAN readings. See updateInputs() */
  private final int REQUIRED_WARMUP_CYCLES = 50;

  public CANCoderIOHardware(AEMCANCoderConfiguration config) {
    this.config = config;

    encoder = CANCoderFactory.createRawWithConfig(config.device, config.configuration);

    positionSignal = encoder.getAbsolutePosition();
    velocitySignal = encoder.getVelocity();

    signals = new BaseStatusSignal[] {positionSignal, velocitySignal};
    BaseStatusSignal.setUpdateFrequencyForAll(100, signals);
  }

  @Override
  public String getName() {
    return getCANDeviceName();
  }

  @Override
  public boolean updateInputs(CANCoderInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    // Do sensor warmup for REQUIRED_WARMUP_CYCLES to ensure our readings are good before using them
    if (warmupReadingCount < REQUIRED_WARMUP_CYCLES) {
      BaseStatusSignal.waitForAll(10.0, signals);
      warmupReadingCount++;
      return false;
    }

    inputs.absolutePositionRotations = positionSignal.getValue().in(Rotations);
    inputs.velocityRotationsPerSecond = velocitySignal.getValue().in(RotationsPerSecond);
    return true;
  }

  @Override
  public boolean setUpdateFrequency(double hz) {
    return BaseStatusSignal.setUpdateFrequencyForAll(hz, signals) == StatusCode.OK;
  }

  @Override
  public CANDeviceID getCANDevice() {
    return this.config.device;
  }
}
