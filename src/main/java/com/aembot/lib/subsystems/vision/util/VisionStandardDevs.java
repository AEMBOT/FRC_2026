package com.aembot.lib.subsystems.vision.util;

public record VisionStandardDevs(
    double xStdDev,
    double yStdDev,
    double zStdDev,
    double rollStdDev,
    double pitchStdDev,
    double yawStdDev) {}
