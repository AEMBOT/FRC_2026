package com.aembot.lib.core.can;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

/** Represents a device that uses CAN for communication and has a CAN ID and CAN BUS */
public class CANDeviceID {
  public enum CANDeviceType {
    TALON_FX("TalonFX"),
    CANCODER("CANcoder"),
    PIGEON2("Pigeon2");

    private final String name;

    private CANDeviceType(String name) {
      this.name = name;
    }

    @Override
    public String toString() {
      return this.name;
    }
  }

  private final String deviceName;
  private final int canID;
  private final String busName;
  private final CANDeviceType deviceType;
  private final String subsystemName;

  /**
   * A {@link StatusSignal} used to determine if the device is streaming data, and therefor
   * connected. <br>
   * </br> The data itself doesn't matter; we just make sure it's being streamed.
   */
  private StatusSignal<?> connectedStatusSignal;

  /** Device we're following, if any */
  private CANDeviceID masterDevice = null;

  /**
   * Create a new CAN Device
   *
   * @param canID ID of this device
   * @param deviceName Name of this device
   * @param subsystemName Name of the subsystem that this CAN device is a part of
   * @param deviceType Type of CAN device that this object is
   * @param busName The bus this device is on
   */
  public CANDeviceID(
      int canID,
      String deviceName,
      String subsystemName,
      CANDeviceType deviceType,
      String busName) {
    this.deviceName = deviceName;
    this.canID = canID;
    this.busName = busName;
    this.deviceType = deviceType;
    this.subsystemName = subsystemName;
  }

  /**
   * Create a new CAN device with the default bus name "rio".
   *
   * @param canID ID of this CAN device
   * @param deviceName Name of this CAN device
   * @param subsystemName Name of the subsystem that this CAN device is a part of
   * @param deviceType Type of CAN device that this object is
   */
  public CANDeviceID(int canID, String deviceName, String subsystemName, CANDeviceType deviceType) {
    this(canID, deviceName, subsystemName, deviceType, "rio");
  }

  public int getDeviceID() {
    return canID;
  }

  public String getBus() {
    return busName;
  }

  public CANDeviceType getDeviceType() {
    return deviceType;
  }

  /** Get the device this device is following, if any */
  public CANDeviceID getMasterDevice() {
    return masterDevice;
  }

  /** Set the device this device is following */
  public void setMasterCANDevice(CANDeviceID master) {
    this.masterDevice = master;
  }

  /**
   * Set status signal to check if the CAN device is connected or not
   *
   * @param signal The signal we are checking to verify can state. The data itself does not matter.
   */
  public void setStatusSignal(StatusSignal<?> signal) {
    connectedStatusSignal = signal;
  }

  /**
   * Set status signal to check if the CAN device is connected or not
   *
   * @param signal The signal we are checking to verify can state. The data itself does not matter.
   * @param hz Frequency at which the signal should be updated in Hz
   */
  public void setStatusSignal(StatusSignal<?> signal, double hz) {
    connectedStatusSignal = signal;
    if (connectedStatusSignal != null) {
      connectedStatusSignal.setUpdateFrequency(hz);
    }
  }

  /**
   * Reference to the CTRE status signal used to determine if the device is connected
   *
   * @return The status signal that this device is connected to
   */
  public StatusSignal<?> getCTREStatusSignal() {
    return connectedStatusSignal;
  }

  /**
   * Check if the device is connected by checking if the status signal set by {@link
   * setStatusSignal} is OK
   */
  public boolean isConnected() {
    if (connectedStatusSignal != null) {
      return connectedStatusSignal.getStatus() == StatusCode.OK;
    }

    return false;
  }

  public boolean equals(CANDeviceID other) {
    return other.canID == canID && other.busName.equals(busName) && other.deviceType == deviceType;
  }

  /**
   * @return a string consisting of GivenName_ID
   */
  public String getDeviceName() {
    return deviceName + "_" + canID;
  }

  /**
   * Get the name of the parent subsystem of which this CAN device is a part of
   *
   * @return Name of top level subsystem
   */
  public String getSubsystemName() {
    return subsystemName;
  }

  @Override
  public String toString() {
    return getDeviceName() + "_" + busName;
  }
}
