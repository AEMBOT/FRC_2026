package com.aembot.lib.phoenix6;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Class for handling CTRE device interactions, including reading errors and applying configurations
 *
 * <p>All methods should retry until max retry is met or the instruction suceeds
 *
 * <p>Amalgamation of Team 254: CTREUtil.java
 * (https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/lib/util/CTREUtil.java)
 * Team 2910: Phoenix6Util.java
 * (https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/phoenix6/Phoenix6Util.java)
 */
public final class CTREUtil {
  public static final int DEFAULT_MAX_RETRIES = 10;

  /**
   * Call a function repeatedly until the function returns true or the maxRetryCount is reached
   *
   * @param tryee The function that is being called to generate a status code that we are basing our
   *     state off
   * @param device The CAN device we are attempting to test on
   * @param maxRetryCount The number of attempts to make on the tryee before failing
   * @return The status code we got last (on success it will be OK)
   */
  public static StatusCode tryUntilOk(Supplier<StatusCode> tryee, int deviceID, int maxRetryCount) {
    StatusCode statusCode = StatusCode.OK;
    for (int i = 0; i < maxRetryCount; ++i) {
      statusCode = tryee.get();
      if (statusCode == StatusCode.OK) break;
    }

    if (statusCode != StatusCode.OK) {
      DriverStation.reportError(
          "Error calling " + tryee + " on ctre device id " + deviceID + ": " + statusCode, true);
    }

    return statusCode;
  }

  /**
   * Set the update frequncy for an array of signals, retrying until successful or the max retries
   * is exceeded
   *
   * @param frequncyHz Frequncy in Hz at which to update the signals
   * @param signals Array of BaseStatusSignal's that we are wanting to set the update rate for
   * @param deviceID CAN device ID of the device that these signals belong to
   * @return The status code we recieved last (on success it will be StatusCode.OK)
   */
  public static StatusCode setUpdateFrequencyForAll(
      double frequncyHz, BaseStatusSignal[] signals, int deviceID) {
    return tryUntilOk(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(frequncyHz, signals),
        deviceID,
        DEFAULT_MAX_RETRIES);
  }

  /** Utilities for configuring CTRE devices */
  public final class Configuration {
    /** Configuration utils for motors */
    public final class Motors {
      /** Enum to represent possible CTRE fault states with string names for easy printing */
      public enum FaultState {
        HARDWARE("Hardware"),
        OVER_SUPPLY_V("OverSupplyVoltage"),
        UNDER_VOLTAGE("Undervoltage"),
        UNSTABLE_SUPPLY_V("UnstableSupplyVoltage"),
        STATOR_CURRENT_LIMIT("StatorCurrentLimit"),
        SUPPLY_CURRENT_LIMIT("SupplyCurrentLimit"),
        UNLICENSED_FEATURE_IN_USE("UnlicensedFeatureInUse"), // :c
        BRIDGE_BROWNOUT("BridgeBrownout"),
        REMOTE_SENSOR_RESET("RemoteSensorReset"),
        REMOTE_SENSOR_POS_OVERFLOW("RemoteSensorPosOverflow"),
        REMOTE_SENSOR_DATA_INVALID("RemoteSensorDataInvalid"),
        FUSED_SENSOR_OUT_OF_SYNC("FusedSensorOutOfSync"),
        USING_FUSED_CANCODER_WHILE_UNLICENSED("UsingFusedCANcoderWhileUnlicensed"),
        MISSING_DIFFERENTIAL_FX("MissingDifferentialFX"),
        REVERSE_HARD_LIMIT("ReverseHardLimit"),
        FORWARD_HARD_LIMIT("ForwardHardLimit"),
        REVERSE_SOFT_LIMIT("ReverseSoftLimit"),
        FORWARD_SOFT_LIMIT("ForwardSoftLimit"),
        PROC_TEMP("ProcTemp"),
        DEVICE_TEMP("DeviceTemp");
        private final String name;

        private FaultState(String name) {
          this.name = name;
        }

        @Override
        public String toString() {
          return this.name;
        }
      }

      /* ------ MotorIOTalonFX ------ */

      /* --- Configuration --- */
      public static StatusCode applyConfiguration(
          MotorIOTalonFX motor, MotorConfiguration<TalonFXConfiguration> config) {
        return applyConfiguration(motor.getTalon(), config);
      }

      public static StatusCode applyConfiguration(
          MotorIOTalonFX motor, TalonFXConfiguration config) {
        return applyConfiguration(motor.getTalon(), config);
      }

      public static StatusCode applyConfigurationNonBlocking(
          MotorIOTalonFX motor, VoltageConfigs config) {
        return applyConfigurationNonBlocking(motor.getTalon(), config);
      }

      public static StatusCode applyConfiguration(
          MotorIOTalonFX motor, HardwareLimitSwitchConfigs config) {
        return applyConfiguration(motor.getTalon(), config);
      }

      public static StatusCode applyConfiguration(MotorIOTalonFX motor, MotionMagicConfigs config) {
        return applyConfiguration(motor.getTalon(), config);
      }

      public static StatusCode applyConfiguration(
          MotorIOTalonFX motor, CurrentLimitsConfigs config) {
        return applyConfiguration(motor.getTalon(), config);
      }

      public static StatusCode getConfiguration(MotorIOTalonFX motor, TalonFXConfiguration config) {
        return refreshConfiguration(motor.getTalon(), config);
      }

      /* --- Error Checking --- */
      public static void checkAndLogFaults(MotorIOTalonFX motor) {
        checkAndLogFaults(motor.getTalon(), motor.getCANDeviceName());
      }

      public static List<FaultState> checkFaults(MotorIOTalonFX motor) {
        return checkFaults(motor.getTalon());
      }

      /* ------- TalonFX------- */

      /* --- Configuration --- */
      public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOk(
            () -> motor.getConfigurator().apply(config), motor.getDeviceID(), DEFAULT_MAX_RETRIES);
      }

      public static StatusCode applyConfigurationNonBlocking(TalonFX motor, VoltageConfigs config) {
        return motor.getConfigurator().apply(config, 0.01);
      }

      public static StatusCode applyConfiguration(
          TalonFX motor, HardwareLimitSwitchConfigs config) {
        return tryUntilOk(
            () -> motor.getConfigurator().apply(config), motor.getDeviceID(), DEFAULT_MAX_RETRIES);
      }

      public static StatusCode applyConfiguration(TalonFX motor, MotionMagicConfigs config) {
        return tryUntilOk(
            () -> motor.getConfigurator().apply(config), motor.getDeviceID(), DEFAULT_MAX_RETRIES);
      }

      public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return tryUntilOk(
            () -> motor.getConfigurator().apply(config), motor.getDeviceID(), DEFAULT_MAX_RETRIES);
      }

      public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOk(
            () -> motor.getConfigurator().refresh(config),
            motor.getDeviceID(),
            DEFAULT_MAX_RETRIES);
      }

      public static StatusCode applyConfiguration(
          TalonFX motor, MotorConfiguration<TalonFXConfiguration> config) {
        return applyConfiguration(motor, config.getMotorConfig());
      }

      /**
       * Optimize the bus utilization for this motor by disabling all signals that have not been
       * marked as needed
       *
       * @param motor The TalonFX we are optimizing
       * @return The status code we recieved last (on success it will be StatusCode.OK)
       */
      public static StatusCode optimizeBusUtilization(TalonFX motor) {
        return tryUntilOk(
            () -> motor.optimizeBusUtilization(), motor.getDeviceID(), DEFAULT_MAX_RETRIES);
      }

      /* --- Error Checking --- */
      /**
       * Get a list of faults present on the motor
       *
       * @param motor The motor we are checking for faults
       */
      public static List<FaultState> checkFaults(TalonFX motor) {
        List<FaultState> states = new ArrayList<>();

        if (motor.getFault_Hardware().getValue()) {
          states.add(FaultState.HARDWARE);
        }
        if (motor.getFault_OverSupplyV().getValue()) {
          states.add(FaultState.OVER_SUPPLY_V);
        }
        if (motor.getFault_Undervoltage().getValue()) {
          states.add(FaultState.UNDER_VOLTAGE);
        }
        if (motor.getFault_UnstableSupplyV().getValue()) {
          states.add(FaultState.UNSTABLE_SUPPLY_V);
        }
        if (motor.getFault_StatorCurrLimit().getValue()) {
          states.add(FaultState.STATOR_CURRENT_LIMIT);
        }
        if (motor.getFault_SupplyCurrLimit().getValue()) {
          states.add(FaultState.SUPPLY_CURRENT_LIMIT);
        }
        if (motor.getFault_UnlicensedFeatureInUse().getValue()) {
          states.add(FaultState.UNLICENSED_FEATURE_IN_USE);
        }
        if (motor.getFault_BridgeBrownout().getValue()) {
          states.add(FaultState.BRIDGE_BROWNOUT);
        }
        if (motor.getFault_RemoteSensorReset().getValue()) {
          states.add(FaultState.REMOTE_SENSOR_RESET);
        }
        if (motor.getFault_RemoteSensorPosOverflow().getValue()) {
          states.add(FaultState.REMOTE_SENSOR_POS_OVERFLOW);
        }
        if (motor.getFault_RemoteSensorDataInvalid().getValue()) {
          states.add(FaultState.REMOTE_SENSOR_DATA_INVALID);
        }
        if (motor.getFault_FusedSensorOutOfSync().getValue()) {
          states.add(FaultState.FUSED_SENSOR_OUT_OF_SYNC);
        }
        if (motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue()) {
          states.add(FaultState.USING_FUSED_CANCODER_WHILE_UNLICENSED);
        }
        if (motor.getFault_MissingDifferentialFX().getValue()) {
          states.add(FaultState.MISSING_DIFFERENTIAL_FX);
        }
        if (motor.getFault_ReverseHardLimit().getValue()) {
          states.add(FaultState.REVERSE_HARD_LIMIT);
        }
        if (motor.getFault_ForwardHardLimit().getValue()) {
          states.add(FaultState.FORWARD_HARD_LIMIT);
        }
        if (motor.getFault_ReverseSoftLimit().getValue()) {
          states.add(FaultState.REVERSE_SOFT_LIMIT);
        }
        if (motor.getFault_ForwardSoftLimit().getValue()) {
          states.add(FaultState.FORWARD_SOFT_LIMIT);
        }
        if (motor.getFault_ProcTemp().getValue()) {
          states.add(FaultState.PROC_TEMP);
        }
        if (motor.getFault_DeviceTemp().getValue()) {
          states.add(FaultState.DEVICE_TEMP);
        }

        return states;
      }

      /**
       * Retrieves a list of faults from a given motor and if there are any it will print an error
       * to the driver station
       *
       * @param motor The motor that the faults are being checked for on
       * @param motorName The name of the motor to allow for better printouts
       */
      public static void checkAndLogFaults(TalonFX motor, String motorName) {
        StringBuilder sb = new StringBuilder();
        for (FaultState state : checkFaults(motor)) {
          sb.append(state.toString()).append(", ");
        }

        if (!sb.isEmpty()) {
          // This'll report the error to StdOut and the driver station (tho I'm not entirely sure
          // how the DS handles it)
          DriverStation.reportError(
              "Talon faults occurred on the following Talon: " + motorName + "! Faults: " + sb,
              false);
        }
      }
    }

    /** Configuration util for encoders */
    public final class Encoders {

      /** Fault states for CTRE Encoders */
      public enum FaultState {
        HARDWARE("Hardware"),
        UNDERVOLTAGE("Undervoltage"),
        BOOT_DURING_ENABLE("BootDuringEnable"),
        UNLICENSED_FEATURE_IN_USE("UnlicensedFeatureInUse"),
        BAD_MAGNET("BadMagnet"), // >:(
        ;
        private final String name;

        private FaultState(String name) {
          this.name = name;
        }

        @Override
        public String toString() {
          return this.name;
        }
      }

      /* ------- CANCoder ------- */

      /* --- Configuration --- */
      public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOk(
            () -> cancoder.getConfigurator().apply(config),
            cancoder.getDeviceID(),
            DEFAULT_MAX_RETRIES);
      }

      /* --- Error Checking --- */
      /**
       * Get a list of faults present on the encoder
       *
       * @param encoder The encoder we are checking for faults
       */
      public static List<FaultState> checkFaults(CANcoder encoder) {
        List<FaultState> states = new ArrayList<>();

        if (encoder.getFault_Hardware().getValue()) {
          states.add(FaultState.HARDWARE);
        }
        if (encoder.getFault_Undervoltage().getValue()) {
          states.add(FaultState.UNDERVOLTAGE);
        }
        if (encoder.getFault_BootDuringEnable().getValue()) {
          states.add(FaultState.BOOT_DURING_ENABLE);
        }
        if (encoder.getFault_UnlicensedFeatureInUse().getValue()) {
          states.add(FaultState.UNLICENSED_FEATURE_IN_USE);
        }
        if (encoder.getFault_BadMagnet().getValue()) {
          states.add(FaultState.BAD_MAGNET);
        }

        return states;
      }

      /**
       * Retrieves a list of faults from a given encoder and if there are any it will print an error
       * to the driver station
       *
       * @param encoder The encoder that the faults are being checked for on
       * @param encoderName The name of the encoder to allow for better printouts
       */
      public static void checkAndLogFaults(CANcoder encoder, String encoderName) {
        StringBuilder sb = new StringBuilder();
        for (FaultState state : checkFaults(encoder)) {
          sb.append(state.toString()).append(", ");
        }

        if (!sb.isEmpty()) {
          DriverStation.reportError(
              "CANcoder faults occurred on the following CANCoder: "
                  + encoderName
                  + "! Faults: "
                  + sb,
              false);
        }
      }
    }
  }
}
