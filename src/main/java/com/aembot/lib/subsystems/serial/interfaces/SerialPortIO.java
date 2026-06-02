package com.aembot.lib.subsystems.serial.interfaces;

import com.aembot.lib.config.subsystems.serial.SerialPortConfiguration;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;

/** 
 * An IO abstraction of the {@link edu.wpi.first.wpilibj.SerialPort} class,
 * Note that, using the base SerialPortIO classes, there is no proper replay implementation.
 * It is recommended to instead use {@link com.aembot.lib.subsystems.serial.io.SerialPortIOSim},
 * which will not provide any input, or to .
 */
public interface SerialPortIO {
  public SerialPortConfiguration getConfig();

  /**
   * Close the SerialPort resource if one is in use. This should only be needed if you're
   * instantiating the IO object in a method scope, which you probably shouldn't be doing.
   */
  public default void close() {}

  

  /**
   * Write raw bytes to the serial port.
   *
   * @param buffer The buffer of bytes to write.
   * @param count The maximum number of bytes to write.
   * @return The number of bytes actually written into the port.
   */
  public int write(byte[] buffer, int count);

  /**
   * Write a string to the serial port.
   *
   * @param data The string to write to the serial port.
   * @return The number of bytes actually written into the port.
   */
  public int writeString(String data);

  /**
   * Force the output buffer to be written to the port.
   *
   * <p>This is used when setWriteBufferMode() is set to kFlushWhenFull to force a flush before the
   * buffer is full.
   */
  public void flush();

  /**
   * Reset the serial port driver to a known state.
   *
   * <p>Empty the transmit and receive buffers in the device and formatted I/O.
   */
  public void reset();

  /**
   * Reapply the config to the SerialPort object if there are external changes. I highly recommend
   * using this class' methods to adjust config values after initial configuration, in which case
   * this method doesn't need to be called externally.
   */
  public void reapplyConfig();

  /**
   * Set the type of flow control to enable on this port.
   *
   * <p>By default, flow control is disabled.
   *
   * @param flowControl the FlowControl m_value to use
   */
  public default void setFlowControl(FlowControl flowControl) {
    this.getConfig().withFlowControl(flowControl);
    this.reapplyConfig();
  }

  /**
   * Enable termination and specify the termination character.
   *
   * <p>Termination is currently only implemented for receive. When the terminator is received, the
   * read() or readString() will return fewer bytes than requested, stopping after the terminator.
   *
   * @param terminator The character to use for termination.
   */
  public default void enableTermination(char terminator) {
    this.getConfig().withTerminationCharacter(terminator);
    this.reapplyConfig();
  }

  /**
   * Enable termination with the default terminator '\n'
   *
   * <p>Termination is currently only implemented for receive. When the terminator is received, the
   * read() or readString() will return fewer bytes than requested, stopping after the terminator.
   *
   * <p>The default terminator is '\n'
   */
  public default void enableTermination() {
    enableTermination('\n');
  }

  /** Disable termination behavior. */
  public default void disableTermination() {
    this.getConfig().withTerminationCharacter(null);
    this.reapplyConfig();
  }

  /**
   * Configure the timeout of the serial m_port.
   *
   * <p>This defines the timeout for transactions with the hardware. It will affect reads if less
   * bytes are available than the read buffer size (defaults to 1) and very large writes.
   *
   * @param timeout The number of seconds to wait for I/O.
   */
  public default void setTimeout(double timeout) {
    this.getConfig().withTimeout(timeout);
    this.reapplyConfig();
  }

  /**
   * Specify the size of the input buffer.
   *
   * <p>Specify the amount of data that can be stored before data from the device is returned to
   * Read. If you want data that is received to be returned immediately, set this to 1.
   *
   * <p>It the buffer is not filled before the read timeout expires, all data that has been received
   * so far will be returned.
   *
   * @param size The read buffer size.
   */
  public default void setReadBufferSize(int size) {
    this.getConfig().withReadBufferSize(size);
    this.reapplyConfig();
  }

  /**
   * Specify the size of the output buffer.
   *
   * <p>Specify the amount of data that can be stored before being transmitted to the device.
   *
   * @param size The write buffer size.
   */
  public default void setWriteBufferSize(int size) {
    this.getConfig().withWriteBufferSize(size);
    this.reapplyConfig();
  }

  /**
   * Specify the flushing behavior of the output buffer.
   *
   * <p>When set to kFlushOnAccess, data is synchronously written to the serial port after each call
   * to either print() or write().
   *
   * <p>When set to kFlushWhenFull, data will only be written to the serial port when the buffer is
   * full or when flush() is called.
   *
   * @param mode The write buffer mode.
   */
  public default void setWriteBufferMode(WriteBufferMode mode) {
    this.getConfig().withWriteBufferMode(mode);
    this.reapplyConfig();
  }
}
