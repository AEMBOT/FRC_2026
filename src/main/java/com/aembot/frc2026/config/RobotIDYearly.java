package com.aembot.frc2026.config;

import java.util.Map;

import com.aembot.lib.config.RobotID;

public enum RobotIDYearly implements RobotID {
    PRODUCTION("Production Bot"),
    ;

    private final String name;
    private String macAddress = null;
    private RobotIDYearly(String name){
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

    @Override
    public Map<String, RobotID> getMACToRobot() {
        return Map.of(
            "blah:blah:blah:blah:blah:blah", RobotIDYearly.PRODUCTION // TODO When we have bot: get actual MAC adress
        );
    }

    @Override
    public RobotID getDefaultRobot() {
        return PRODUCTION;
    }
}
