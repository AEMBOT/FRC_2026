package com.aembot.lib.subsystems.leds.io;

import com.aembot.lib.subsystems.leds.interfaces.LEDStripIO;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class LEDStripIOReal
    implements LEDStripIO { // this does feel kind of silly w/ how short it is
  public static final int BAUD_RATE = 115200;

  private final SerialPort kSerialPort;

  public LEDStripIOReal(Port port) {
    this.kSerialPort = new SerialPort(BAUD_RATE, port);
  }

  @Override
  public void sendCode(char code) {
    this.kSerialPort.writeString(Character.toString(code));
  }
}
