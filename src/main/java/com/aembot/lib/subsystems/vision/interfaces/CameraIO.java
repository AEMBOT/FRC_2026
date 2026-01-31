package com.aembot.lib.subsystems.vision.interfaces;

import com.aembot.lib.config.camera.CameraConfiguration;
import com.aembot.lib.subsystems.vision.VisionInputs;

/** Basic camera object */
public interface CameraIO {

  /**
   * @return The configuration for this camera
   */
  public CameraConfiguration getConfiguration();

  /**
   * Update the state of this camera
   *
   * @param inputs Vision inputs to be updated
   */
  public void updateInputs(VisionInputs inputs);
}
