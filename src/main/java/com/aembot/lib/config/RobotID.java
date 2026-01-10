package com.aembot.lib.config;

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
}
