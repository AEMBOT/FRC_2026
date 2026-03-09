package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.tracing.Traced;

public class AprilCameraReplayIO implements AprilCameraIO {
  private CameraConfiguration config;

  public AprilCameraReplayIO(CameraConfiguration config) {
    this.config = config;
  }

  @Override
  public CameraConfiguration getConfiguration() {
    return config;
  }

  @Override
  @Traced(category = "Vision")
  public void updateInputs(AprilVisionInputs inputs) {}

  @Override
  @Traced(category = "Vision")
  public void updateNetworkTablesForDisabled() {}

  @Override
  @Traced(category = "Vision")
  public void updateNetworkTablesForEnabled() {}
}
