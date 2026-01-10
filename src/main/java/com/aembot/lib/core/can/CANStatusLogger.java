package com.aembot.lib.core.can;

import com.aembot.lib.core.logging.Loggable;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Manages logging for a can bus and its attached devices. <br>
 * </br> There can be one instance of {@link CANStatusLogger} per CAN bus. <br>
 * </br> Every instance is stored in the static {@code Map<String canBus, CANStatusLogger instance>}
 * {@code instances}, which can be queried via the static {@code get} method
 */
public class CANStatusLogger implements Loggable {
  /* --- STATIC FIELDS & METHODS --- */

  /** Map between CANBus and instance */
  private static Map<String, CANStatusLogger> instances = new HashMap<String, CANStatusLogger>();

  /**
   * Retrieve the status logger for the given CAN bus name, creating it if it doesn't exist yet.
   *
   * @return Reference to the desired {@link CANStatusLogger}
   */
  public static CANStatusLogger get(String busName) {
    CANStatusLogger logger = instances.get(busName);
    if (logger == null) {
      logger = new CANStatusLogger(busName);
      instances.put(busName, logger);
    }

    return logger;
  }

  /** Update the logs for all CAN status loggers. Does nothing while enabled */
  public static void updateAllLogs() {
    if (DriverStation.isDisabled()) { // Only update CAN statuses when the robot is disabled
      for (CANStatusLogger logger : instances.values()) {
        logger.updateLog("", "");
      }
    }
  }

  /* ---- INSTANCE FIELDS & METHODS ---- */
  /** List of devices that should be tracked */
  private List<CANDeviceID> devices = new ArrayList<>();

  /** The can bus we are logging data for */
  private final CANBus canBus;

  private final String logLocationPrefix;

  /**
   * List of signals that need to be checked for this CAN logger. Automatically populated by {@code
   * populateCTRESignals()}
   */
  private BaseStatusSignal[] ctreSignals = null;

  /**
   * Setup a new CAN status logger on the desired bus
   *
   * @param busName Name of the bus we are performing logging for
   */
  private CANStatusLogger(String busName) {
    this.canBus = new CANBus(busName);

    // We are logging can status to CANStatus/<bus name>/*
    logLocationPrefix = "CANStatus/" + busName + "/";
  }

  public String getBusName() {
    return this.canBus.getName();
  }

  /**
   * Register a new CAN device with the CAN logger
   *
   * @param device The device to register with the logger
   */
  public void registerCANDevice(CANDeviceID device) {
    devices.add(device);
  }

  // TODO Add helper method for registering drivetrain devices

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    logBusStatus();
    logDeviceStatuses();
  }

  /**
   * Re-populate {@code ctreSignals} array with signals from all registered devices that provide one
   */
  private void populateCTRESignals() {
    int validCTRESignalCount = 0;
    for (CANDeviceID device : devices) {
      if (device.getCTREStatusSignal() != null) {
        validCTRESignalCount++;
      }
    }

    if (ctreSignals == null || ctreSignals.length != validCTRESignalCount) {
      ctreSignals = new BaseStatusSignal[validCTRESignalCount];
      int index = 0;
      for (CANDeviceID device : devices) {
        if (device.getCTREStatusSignal() != null) {
          ctreSignals[index++] = device.getCTREStatusSignal();
        }
      }
    }
  }

  /** Log the data on this instance's canBus */
  private void logBusStatus() {
    CANBusStatus status = canBus.getStatus();
    Logger.recordOutput(logLocationPrefix + "BusStatus", status.Status);
    Logger.recordOutput(logLocationPrefix + "BusUtilization", status.BusUtilization);
  }

  /**
   * Log statuses from registered devices with valid status signals. Automatically calls {@code
   * populateCTRESignals()} if needed.
   */
  private void logDeviceStatuses() {
    if (ctreSignals == null) {
      populateCTRESignals();
    }

    // We don't really care about the result of this as we still want to update the status.
    // THIS MUST HAPPEN HERE (isConnected doesn't update base status signals)
    BaseStatusSignal.refreshAll(ctreSignals);

    for (CANDeviceID device : devices) {
      String deviceName = device.getDeviceName();
      String subsystemName = device.getSubsystemName();
      boolean isConnected = device.isConnected();

      Logger.recordOutput(
          logLocationPrefix
              + subsystemName
              + "/"
              + device.getDeviceType().toString()
              + "/"
              + deviceName,
          isConnected);

      if (device.getMasterDevice() != null) {
        Logger.recordOutput(
            logLocationPrefix
                + subsystemName
                + "/"
                + device.getDeviceType().toString()
                + "/"
                + deviceName
                + "/Following",
            device.getMasterDevice().getDeviceName());
      }
    }
  }
}
