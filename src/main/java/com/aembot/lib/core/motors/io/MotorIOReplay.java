package com.aembot.lib.core.motors.io;

import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import edu.wpi.first.math.Pair;

public class MotorIOReplay implements MotorIO {

  public String getName() {
    return "Replay";
  }
  ;

  public boolean updateInputs(MotorInputs inputs) {
    return true;
  }
  ;

  public boolean setEnableSoftwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
    return true;
  }
  ;

  public Pair<Boolean, Boolean> getEnableSoftwareLimits() {
    return new Pair<Boolean, Boolean>(true, true);
  }
  ;

  public boolean setEnableHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
    return true;
  }
  ;

  public Pair<Boolean, Boolean> getEnableHardwareLimits() {
    return new Pair<Boolean, Boolean>(true, true);
  }
  ;

  public boolean setZeroOnHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
    return true;
  }
  ;

  public Pair<Boolean, Boolean> getZeroOnHardwareLimits() {
    return new Pair<Boolean, Boolean>(true, true);
  }
  ;

  public boolean follow(CANDeviceID masterDevice, FollowDirection direction) {
    return true;
  }
  ;

  public void setSmartMotorConfig(MotionMagicConfigs config) {}
  ;

  public void setVoltageConfig(VoltageConfigs config) {}
  ;

  public boolean setCurrentEncoderPosition(double position) {
    return true;
  }
  ;

  public boolean setVoltageOutput(double volts) {
    return true;
  }
  ;

  public boolean setTorqueCurrent(double current) {
    return true;
  }
  ;

  public boolean setOpenLoopDutyCycle(double dutyCycle) {
    return true;
  }
  ;

  public boolean setNeutralMode(NeutralMode mode) {
    return true;
  }
  ;

  public boolean setPIDPositionSetpoint(double positionUnits, int slot) {
    return true;
  }
  ;

  public boolean setPIDVelocitySetpoint(double velocityUnitsPerSecond, int slot) {
    return true;
  }
  ;

  public boolean setSmartPositionSetpoint(double positionUnits, int slot) {
    return true;
  }
  ;

  public boolean setSmartVelocitySetpoint(double velocityUnitsPerSecond, int slot) {
    return true;
  }
  ;

  public boolean setDynamicSmartPositionSetpoint(
      double positionUnits,
      double velocity,
      double acceleration,
      double jerk,
      double feedforward,
      int slot) {
    return true;
  }
  ;
}
