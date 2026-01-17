package com.aembot.frc2026.config;

import com.aembot.lib.config.RobotID;
import com.aembot.lib.core.network.NetworkUtils;
import java.util.Map;

public enum RobotIDYearly implements RobotID {
  PRODUCTION("Production Bot"),
  ;

  private final String name;
  private String macAddress = null;

  private RobotIDYearly(String name) {
    this.name = name;
  }

  @Override
  public RobotID withMACAddress(String mac) {
    this.macAddress = mac;
    return this;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String getMACAddress() {
    return macAddress;
  }

  private static final Map<String, RobotID> ROBOT_TO_MAC =
      Map.of("blah:blah:blah:blah:blah:blah", RobotIDYearly.PRODUCTION);

  private static final RobotID DEFAULT_ROBOT = PRODUCTION;

  /**
   * Static method used to ascertain the current robot that this code is being run on by comparing
   * the HW MAC address to a known one
   */
  public static RobotID getIdentification() {
    String macAddress = NetworkUtils.MAC.getMACAddress();
    
    return ROBOT_TO_MAC.getOrDefault(macAddress, DEFAULT_ROBOT).withMACAddress(macAddress);
  }
}
