package com.aembot.lib.subsystems.serial;

import com.aembot.lib.config.subsystems.serial.SerialPortConfiguration;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.subsystems.serial.interfaces.SerialPortIO;

public class SerialDevice implements Loggable {
  protected final SerialInputs kInputs = new SerialInputs();
  protected final SerialPortConfiguration kConfig;
  protected final SerialPortIO kIO;

  /**
   * Get the number of bytes currently available to read from the serial port.
   *
   * @return The number of bytes available to read.
   */
  public int getBytesReceived() {
    
  }

  /**
   * Read a string out of the buffer. Reads the entire contents of the buffer
   *
   * @return The read string
   */
  public String readString();

  /**
   * Read a string out of the buffer. Reads the entire contents of the buffer
   *
   * @param count the number of characters to read into the string
   * @return The read string
   */
  public String readString(int count);

  /**
   * Read raw bytes out of the buffer.
   *
   * @param count The maximum number of bytes to read.
   * @return An array of the read bytes
   */
  public byte[] read(final int count);

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateLog'");
  }
}
