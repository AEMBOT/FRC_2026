package com.aembot.lib.core.motors.factories;

import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.CANStatusLogger;
import com.aembot.lib.core.phoenix6.CTREUtil;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

/** Factory methods for creating TalonFXs */
public class TalonFXFactory {
  /**
   * Create a new TalonFX with the talon FX configuration supplied
   *
   * @param device CANDevice the represents the Talon being created
   * @param config The config that should be applied to the talon after its created
   * @return The newly created Talon instance
   */
  public static TalonFX createRawWithConfig(CANDeviceID device, TalonFXConfiguration config) {
    TalonFX talon = createRaw(device);
    CTREUtil.Configuration.Motors.applyConfiguration(talon, config);

    // Set update rate of our CANDeviceID status signal to update at 100 hz
    device.setStatusSignal(talon.getSupplyVoltage(), 100);

    // Automatically register the Talon with the CAN status logger upon creation
    CANStatusLogger.get(device.getBus()).registerCANDevice(device);
    return talon;
  }

  /**
   * Create a new TalonFX and does not apply a configuration
   *
   * @param device CANDevice the represents the Talon being created
   * @return The newly created Talon instance
   */
  public static TalonFX createRawNoConfig(CANDeviceID device) {
    TalonFX talon = createRaw(device);

    // Set update rate of our CANDeviceID status signal to update at 100 hz
    device.setStatusSignal(talon.getSupplyVoltage(), 100);

    // Automatically register the Talon with the CAN status logger upon creation
    CANStatusLogger.get(device.getBus()).registerCANDevice(device);
    return talon;
  }

  /**
   * Create a new TalonFX with the default configuration described below and link it with the given
   * CANDeviceID
   *
   * @param device CANDeviceID that the TalonFX is object is being created from
   * @return The newly created Talon FX
   */
  public static TalonFX createRawDefault(CANDeviceID device) {
    return createRawWithConfig(device, getDefaultConfig());
  }

  /**
   * Retrieve the default TalonFX configuration we should use when we call createDefault
   *
   * @return Default TalonFX config
   */
  public static TalonFXConfiguration getDefaultConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            // Configure motor output parameters
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withDutyCycleNeutralDeadband(0.04)
                    .withPeakForwardDutyCycle(1.0)
                    .withPeakReverseDutyCycle(-1.0))

            // Configure current limits
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(false)
                    .withStatorCurrentLimitEnable(false))

            // Configure software limit switches
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withForwardSoftLimitThreshold(0.0)
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(0.0))

            // Configure hardware limit switches
            .withHardwareLimitSwitch(
                new HardwareLimitSwitchConfigs()
                    // Forward hardware limit switch
                    .withForwardLimitEnable(false)
                    .withForwardLimitAutosetPositionEnable(false)
                    .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
                    .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)

                    // Reverse hardware limit switch
                    .withReverseLimitEnable(false)
                    .withReverseLimitAutosetPositionEnable(false)
                    .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
                    .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen))

            // Setup feedback sources
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withFeedbackRotorOffset(0.0)
                    .withSensorToMechanismRatio(1.0))

            // Setup audio
            .withAudio(new AudioConfigs().withBeepOnBoot(true));

    return config;
  }

  /**
   * Create a new TalonFX
   *
   * @param device CAN device that represents this talon
   * @return The newly created TalonFX
   */
  private static TalonFX createRaw(CANDeviceID device) {
    TalonFX talon = new TalonFX(device.getDeviceID(), device.getBus());
    talon.clearStickyFaults();
    return talon;
  }
}
