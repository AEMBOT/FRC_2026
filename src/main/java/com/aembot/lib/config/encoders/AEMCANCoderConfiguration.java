package com.aembot.lib.config.encoders;

import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

/** Configuration describing the setup of a CAN coder */
public class AEMCANCoderConfiguration {
  /** ID of the configured CANCoder */
  public CANDeviceID device;

  /** Underlying ctre config */
  public CANcoderConfiguration configuration;
}
