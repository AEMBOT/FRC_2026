package com.aembot.lib.subsystems.serial;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SerialInputs implements LoggableInputs {
  /** circular buffer storing recieved bytes. */
  public byte[] byteBuffer = new byte[0];

  @Override
  public void toLog(LogTable table) {
    table.put("RecievedBytes", byteBuffer);
  }

  @Override
  public void fromLog(LogTable table) {
    byteBuffer = table.get("RecievedBytes", byteBuffer);
  }
}
