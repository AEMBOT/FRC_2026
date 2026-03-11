package com.aembot.lib.core.encoders.io;

import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;

/** Replay implementation for CANCoder IO */
public class CANCoderIOReplay implements CANCoderIO {

  @Override
  public String getName() {
    return "Replay";
  }

  @Override
  public boolean updateInputs(CANCoderInputs inputs) {
    return true;
  }

  @Override
  public boolean setUpdateFrequency(double hz) {
    return true;
  }

  @Override
  public double getRawAngle() {
    return 0.0;
  }
}
