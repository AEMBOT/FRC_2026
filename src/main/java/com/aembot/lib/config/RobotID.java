package com.aembot.lib.config;

import com.aembot.lib.core.network.NetworkUtils;
import java.util.Map;

/** Interface to describe a robot. Should be implemented by an enum. */
public interface RobotID {
  /**
   * Builder method to add a MACAdress to the robot
   *
   * @return itself
   */
  public RobotID withMACAddress(String mac);

  public String getName();

  public String getMACAddress();

  /** Get a map of string-formatted MAC adresses to {@link RobotID}s */
  public Map<String, RobotID> getMACToRobot();

  /**
   * Get the default {@link RobotID} for this season for when a MAC adress isn't present in {@code
   * getRobotToMac} or the MAC address is null. <br>
   * </br> This will probably be used in sim also. TODO test
   *
   * @return the default RobotID, without a MAC address attached.
   */
  public RobotID getDefaultRobot();

  /**
   * Static method used to ascertain the current robot that this code is being run on by comparing
   * the HW MAC address to a known one
   */
  public default RobotID getIdentification() {
    String macAddress = NetworkUtils.MAC.getMACAddress();
    if (macAddress == null) {
      return getDefaultRobot().withMACAddress("NULL MAC ADDRESS");
    }
    RobotID id = getMACToRobot().get(macAddress).withMACAddress(macAddress);
    // Default to the main robot if MAC was unable to be retrieved
    return (id != null) ? id : getDefaultRobot().withMACAddress(macAddress);
  }
}
