package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;

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
  @Traced
  public void updateInputs(AprilVisionInputs inputs) {}

  @Override
  @Traced
  public void updateNetworkTablesForDisabled() {}

  @Override
  @Traced
  public void updateNetworkTablesForEnabled() {}
}
