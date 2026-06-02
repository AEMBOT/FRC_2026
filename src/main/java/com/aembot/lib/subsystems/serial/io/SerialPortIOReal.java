package com.aembot.lib.subsystems.serial.io;

import com.aembot.lib.config.subsystems.serial.SerialPortConfiguration;
import com.aembot.lib.subsystems.serial.interfaces.SerialPortIO;
import edu.wpi.first.wpilibj.SerialPort;

public class SerialPortIOReal implements SerialPortIO {
  private final SerialPortConfiguration kConfig;
  private final SerialPort kSerialObject;

  public SerialPortIOReal(SerialPortConfiguration config) {
    this.kConfig = config;
    this.kSerialObject = config.createSerialPort();
  }

  @Override
  public SerialPortConfiguration getConfig() {
    return kConfig;
  }

  @Override
  public int getBytesReceived() {
    return kSerialObject.getBytesReceived();
  }

  @Override
  public String readString() {
    return kSerialObject.readString();
  }

  @Override
  public String readString(int count) {
    return kSerialObject.readString(count);
  }

  @Override
  public byte[] read(int count) {
    return kSerialObject.read(count);
  }

  @Override
  public int write(byte[] buffer, int count) {
    return kSerialObject.write(buffer, count);
  }

  @Override
  public int writeString(String data) {
    return kSerialObject.writeString(data);
  }

  @Override
  public void flush() {
    kSerialObject.flush();
  }

  @Override
  public void reset() {
    kSerialObject.reset();
  }

  @Override
  public void reapplyConfig() {
    kConfig.reapplyConfig(kSerialObject);
  }
}
