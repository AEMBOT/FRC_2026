package com.aembot.lib.subsystems.aprilvision.io;

import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.subsystems.aprilvision.AprilVisionInputs;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.function.BiConsumer;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * IO layer for simulating a limelight. Extends {@link Limelight4IOHardware}, publishing data from
 * PhotonVision's sim to the Limelight's expected network table.
 */
public class Limelight4IOSim extends Limelight4IOHardware {
  private final SimulatedCameraConfiguration simConfig;

  private final PhotonCamera photonCamera;
  private final PhotonCameraSim photonCameraSim;

  /**
   * Constructor go brrr
   *
   * @param config Config for the simulated camera
   * @param fieldConstants This season's field constants
   * @param robotStateInstance This year's instance of {@link RobotState}
   * @param registerVisionSimulationConsumer A consumer for registering the simulated photonvision
   *     camera with the vision simulation system
   */
  public Limelight4IOSim(
      SimulatedCameraConfiguration config,
      YearFieldConstantable fieldConstants,
      RobotState robotStateInstance,
      BiConsumer<PhotonCameraSim, Transform3d> registerVisionSimulationConsumer) {
    super(config.cameraConfiguration, fieldConstants, robotStateInstance);
    this.simConfig = config;

    this.photonCamera = new PhotonCamera(simConfig.cameraConfiguration.cameraName);
    this.photonCameraSim = new PhotonCameraSim(photonCamera, simConfig.simCameraProperties);

    registerVisionSimulationConsumer.accept(
        photonCameraSim,
        new Transform3d(
            cameraConfiguration.getCameraPosition().getTranslation(),
            cameraConfiguration.getCameraPosition().getRotation()));
  }

  @Override
  public void updateInputs(AprilVisionInputs inputs) {
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    if (results.size() > 0) {
      PhotonPipelineResult result = results.get(results.size() - 1);

      // Not sure if this is the best way to do latency
      captureLatencyEntry.setDouble(Timer.getFPGATimestamp() - result.getTimestampSeconds());

      PhotonTrackedTarget target = result.getBestTarget();

      validTagEntry.setBoolean(target != null);
      if (target != null) {
        int tagID = target.getFiducialId();
        tagIDEntry.setInteger(tagID);

        double xOffset = target.getYaw();
        xOffsetEntry.setDouble(xOffset);

        List<TargetCorner> targetCorners = target.getDetectedCorners();

        double[] publishedArray = new double[8];
        if (targetCorners.size() == 4) {
          for (int i = 0; i < 4; i++) {
            publishedArray[i * 2] = targetCorners.get(i).x;
            publishedArray[i * 2 + 1] = targetCorners.get(i).y;
          }
        }

        tagCornerPositionsEntry.setDoubleArray(publishedArray);
      }
    }

    super.updateInputs(inputs);
  }
}
