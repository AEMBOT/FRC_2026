package com.aembot.lib.config.subsystems.serial;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import java.util.ArrayList;
import java.util.List;

/**
 * Object used to configure the behavior of a serial port. Note that some values can changed
 * post-configuration via IO methods.
 */
public class SerialPortConfiguration {
  public final String kName;

  public Integer kBaudRate;
  public Port kPort;

  /** The number of data bits per transfer. Valid values are between 5 and 8 bits. Defaults to 8. */
  public int kDataBits = 8;

  /** Select the type of parity checking to use. Defaults to {@link Parity#kNone}. */
  public Parity kParity = Parity.kNone;

  /**
   * The number of stop bits to use as defined by the enum {@link StopBits}. Defaults to {@link
   * StopBits#kOne}
   */
  public StopBits kStopBits = StopBits.kOne;

  /**
   * The type of flow control to enable on this port. Defaults to {@link FlowControl#kNone}
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   */
  public FlowControl kFlowControl = FlowControl.kNone;

  /**
   * Termination is currently only implemented for receive. When the terminator is received, the
   * read() or readString() will return fewer bytes than requested, stopping after the terminator.
   *
   * <p>Setting to {@code null} is equivalent to {@link SerialPort#disableTermination()}.
   *
   * <p>Defaults to {@code null}.
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   */
  public Character kTerminationCharacter = null;

  /**
   * This defines the timeout in seconds for transactions with the hardware. It will affect reads if
   * less bytes are available than the read buffer size (defaults to 1) and very large writes.
   * Defaults to 5 seconds.
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   */
  public double kTimeout = 5;

  /**
   * The amount of data that can be stored before data from the device is returned to Read. If you
   * want data that is received to be returned immediately, set this to 1.
   *
   * <p>If the buffer is not filled before the read timeout expires, all data that has been received
   * so far will be returned.
   */
  public int kReadBufferSize = 1;

  /**
   * The amount of data that can be stored before being transmitted to the device. (I _think_ this
   * only has an effect while {@link #kWriteBufferMode} is {@link WriteBufferMode#kFlushOnAccess},
   * but I'm not sure)
   *
   * <p>Leave null for default (_might_ have UB; default isn't accessible thru the {@link
   * SerialPort} class)
   */
  public Integer kWriteBufferSize = null;

  /**
   * The flushing behavior of the output buffer.
   *
   * <p>When set to kFlushOnAccess, data is synchronously written to the serial port after each call
   * to either print() or write().
   *
   * <p>When set to kFlushWhenFull, data will only be written to the serial port when the buffer is
   * full or when flush() is called.
   */
  public WriteBufferMode kWriteBufferMode = WriteBufferMode.kFlushOnAccess;

  public SerialPortConfiguration(String name) {
    this.kName = name;
  }

  /**
   * Set the port this configuration corresponds to.
   *
   * @param port
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withPort(Port port) {
    this.kPort = port;
    return this;
  }

  /**
   * Set the baud rate used for communication over this serial port
   *
   * @param baudRate
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withBaudRate(int baudRate) {
    this.kBaudRate = baudRate;
    return this;
  }

  /**
   * The number of data bits per transfer. Valid values are between 5 and 8 bits. Defaults to 8.
   *
   * @param dataBits
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withDataBits(int dataBits) {
    this.kDataBits = dataBits;
    return this;
  }

  /**
   * Select the type of parity checking to use. Defaults to {@link Parity#kNone}.
   *
   * @param parity
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withParity(Parity parity) {
    this.kParity = parity;
    return this;
  }

  /**
   * The number of stop bits to use as defined by the enum {@link StopBits}. Defaults to {@link
   * StopBits#kOne}
   *
   * @param stopBits
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withStopBits(StopBits stopBits) {
    this.kStopBits = stopBits;
    return this;
  }

  /**
   * The type of flow control to enable on this port. Defaults to {@link FlowControl#kNone}
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   *
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withFlowControl(FlowControl flowControl) {
    this.kFlowControl = flowControl;
    return this;
  }

  /**
   * Termination is currently only implemented for receive. When the terminator is received, the
   * read() or readString() will return fewer bytes than requested, stopping after the terminator.
   *
   * <p>Setting to {@code null} is equivalent to {@link SerialPort#disableTermination()}.
   *
   * <p>Defaults to {@code null}.
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   *
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withTerminationCharacter(Character character) {
    this.kTerminationCharacter = character;
    return this;
  }

  /**
   * This defines the timeout in seconds for transactions with the hardware. It will affect reads if
   * less bytes are available than the read buffer size (defaults to 1) and very large writes.
   * Defaults to 5 seconds.
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   *
   * @param timeoutSeconds
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withTimeout(double timeoutSeconds) {
    return setTimeout(timeoutSeconds);
  }

  /**
   * This defines the timeout in seconds for transactions with the hardware. It will affect reads if
   * less bytes are available than the read buffer size (defaults to 1) and very large writes.
   * Defaults to 5 seconds.
   *
   * <p><strong>Can be modified by IO methods (preferred method after initial config)
   *
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration setTimeout(double timeoutSeconds) {
    this.kTimeout = timeoutSeconds;
    return this;
  }

  /**
   * The amount of data that can be stored before data from the device is returned to Read. If you
   * want data that is received to be returned immediately, set this to 1.
   *
   * <p>If the buffer is not filled before the read timeout expires, all data that has been received
   * so far will be returned.
   *
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withReadBufferSize(int size) {
    this.kReadBufferSize = size;
    return this;
  }

  /**
   * The amount of data that can be stored before being transmitted to the device. (I _think_ this
   * only has an effect while {@link #kWriteBufferMode} is {@link WriteBufferMode#kFlushOnAccess},
   * but I'm not sure)
   *
   * <p>Leave null for default (_might_ have UB; default isn't accessible thru the {@link
   * SerialPort} class)
   *
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withWriteBufferSize(int size) {
    this.kWriteBufferSize = size;
    return this;
  }

  /**
   * The flushing behavior of the output buffer.
   *
   * <p>When set to kFlushOnAccess, data is synchronously written to the serial port after each call
   * to either print() or write().
   *
   * <p>When set to kFlushWhenFull, data will only be written to the serial port when the buffer is
   * full or when flush() is called.
   *
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration withWriteBufferMode(WriteBufferMode mode) {
    this.kWriteBufferMode = mode;
    return this;
  }

  /**
   * Create a {@link SerialPort} object with this configuration. Intended to be used by an IO layer.
   *
   * @return The created {@link SerialPort}
   */
  public SerialPort createSerialPort() {
    return reapplyConfig(new SerialPort(kBaudRate, kPort, kDataBits, kParity, kStopBits));
  }

  /**
   * Reapply config values to the given {@link SerialPort}.
   *
   * <p>Immutable values that will <strong>not</strong> be reapplied include:
   *
   * <ul>
   *   <li>{@link #kBaudRate}
   *   <li>{@link #kPort}
   *   <li>{@link #kDataBits}
   *   <li>{@link #kParity}
   *   <li>{@link #kStopBits}
   * </ul>
   *
   * @param target The {@link SerialPort} to apply config to.
   * @return target
   */
  public SerialPort reapplyConfig(SerialPort target) {
    target.setFlowControl(kFlowControl);
    target.setTimeout(kTimeout);
    target.setReadBufferSize(kReadBufferSize);
    target.setWriteBufferMode(kWriteBufferMode);

    if (kWriteBufferSize != null) target.setWriteBufferSize(kWriteBufferSize);

    if (kTerminationCharacter == null) {
      target.disableTermination();
    } else {
      target.enableTermination(kTerminationCharacter);
    }

    return target;
  }

  /**
   * Make this config object essentially a deep copy of the given config object. The primary use of
   * this is keeping two seperate instances of the config in sim to simulate application ({@link
   * #reapplyConfig(SerialPort)}).
   *
   * @param source The config to pull values from
   * @return This {@link SerialPortConfiguration} for chaining.
   */
  public SerialPortConfiguration pullValuesFrom(SerialPortConfiguration source) {
    return this.withPort(source.kPort)
        .withBaudRate(source.kBaudRate)
        .withDataBits(source.kDataBits)
        .withParity(source.kParity)
        .withStopBits(source.kStopBits)
        .withFlowControl(source.kFlowControl)
        .withTimeout(source.kTimeout)
        .withReadBufferSize(source.kReadBufferSize)
        .withTerminationCharacter(source.kTerminationCharacter)
        .withWriteBufferMode(source.kWriteBufferMode)
        .withWriteBufferSize(source.kWriteBufferSize);
  }

  /**
   * Check that all values required for a serial port are set on this config. If they are not, throw
   * a {@link VerifyError}. Intended to be called at the end of an initialization chain.
   *
   * @return this {@link SerialPortConfiguration} for chaining
   */
  public SerialPortConfiguration validate() {
    List<String> errors = new ArrayList<>();

    List<String> missing = new ArrayList<>();
    if (this.kBaudRate == null) missing.add("kBaudRate");
    if (this.kPort == null) missing.add("kPort");

    if (missing.size() != 0) {
      errors.add("Config for this serial port does not have a set " + String.join(",", missing));
      throw new VerifyError(
          "Config for this serial port does not have a set " + String.join(",", missing));
    }

    if (this.kDataBits > 8 || this.kDataBits < 5)
      errors.add("kDataBits must be between 5 and 8 (inclusive)");

    if (errors.size() != 0) {
      throw new VerifyError(String.join(";", errors));
    }

    return this;
  }
}
