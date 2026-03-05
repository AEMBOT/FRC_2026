package com.aembot.lib.core.can.interfaces;

import com.aembot.lib.core.can.CANDeviceID;

/**
 * Interface for classes with attached {@link CANDeviceID}s <br>
 * </br> (Might eat other interfaces)
 */
public interface CANable {
  /** Get the {@link CANDeviceID} associated with this object. */
  public CANDeviceID getCANDevice();

  /** Gets the CAN ID of the {@link CANDeviceID} associated with this object. */
  public default int getCANDeviceID() {
    return getCANDevice().getDeviceID();
  }

  /** Gets the name of the {@link CANDeviceID} associated with this object. */
  public default String getCANDeviceName() {
    return getCANDevice().getDeviceName();
  }
}
