package com.aembot.lib.core.encoders.io;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.interfaces.CANable;
import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.encoders.factories.CANCoderFactory;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class CANCoderSimIO implements CANCoderIO, CANable {

  private class SimulatedCANCoderState {
    double encoderPosition;
    double encoderVelocity;
    double encoderAcceleration;
  }

  protected final CANcoder encoder;

  protected final CANcoderSimState simState;

  protected final AEMCANCoderConfiguration config;

  protected double lastUpdateTimestamp = 0.0;

  protected SimulatedCANCoderState inputs;

  public CANCoderSimIO(AEMCANCoderConfiguration config) {
    this.config = config;
    encoder = CANCoderFactory.createRawWithConfig(config.device, config.configuration);

    simState = encoder.getSimState();

    lastUpdateTimestamp = Timer.getFPGATimestamp();
  }

  public void updateSimState(double position, double velocity, double acceleration) {
    double absolutePositionRotations = position % 1.0;

    inputs.encoderPosition = absolutePositionRotations;
    inputs.encoderVelocity = velocity;
    inputs.encoderAcceleration = acceleration;

    simState.setRawPosition(inputs.encoderPosition);
    simState.setVelocity(inputs.encoderVelocity);
  }

  public void logSim(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(
        standardPrefix + "/Simulation/" + getName() + "/PositionRotations", inputs.encoderPosition);
    Logger.recordOutput(
        standardPrefix + "/Simulation/" + getName() + "/VelocityRotationsPerSec",
        inputs.encoderVelocity);
    Logger.recordOutput(
        standardPrefix + "/Simulation/" + getName() + "/AccelerationRotationsPerSecSq",
        inputs.encoderAcceleration);
  }

  @Override
  public CANDeviceID getCANDevice() {
    return config.device;
  }

  @Override
  public String getName() {
    return getCANDeviceName();
  }

  @Override
  public boolean updateInputs(CANCoderInputs inputs) {

    inputs.absolutePositionRotations = this.inputs.encoderPosition;
    inputs.velocityRotationsPerSecond = this.inputs.encoderVelocity;

    return true;
  }

  @Override
  public boolean setUpdateFrequency(double hz) {
    return true;
  }

  @Override
  public double getRawAngle() {
    return inputs.encoderPosition;
  }
}
