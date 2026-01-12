package com.aembot.lib.core.encoders.factories;

import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.CANStatusLogger;
import com.aembot.lib.core.phoenix6.CTREUtil;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANCoderFactory {
  /**
   * Create a new raw CANcoder with the CANcoderConfiguration supplied
   *
   * @param device {@link CANDeviceID} representing the CANcoder being created
   * @param config The config that should be applied to the CANcoder after it's created
   * @return The newly created CANcoder instance
   */
  public static CANcoder createRawWithConfig(CANDeviceID device, CANcoderConfiguration config) {
    CANcoder encoder = createRaw(device);

    CTREUtil.Configuration.Encoders.applyConfiguration(encoder, config);

    device.setStatusSignal(encoder.getSupplyVoltage(), 100);

    // Automatically register the encoder with the CAN status logger upon creation
    CANStatusLogger.get(device.getBus()).registerCANDevice(device);
    return encoder;
  }

  /**
   * Create a new raw CANcoder
   *
   * @param device CAN device that represents this CANcoder
   * @return The newly created CANcoder
   */
  private static CANcoder createRaw(CANDeviceID device) {
    CANcoder encoder = new CANcoder(device.getDeviceID(), device.getBus());
    encoder.clearStickyFaults();
    return encoder;
  }
}
