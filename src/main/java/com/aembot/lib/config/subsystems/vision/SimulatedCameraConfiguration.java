package com.aembot.lib.config.subsystems.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SimulatedCameraConfiguration {
    public final SimCameraProperties photonSimCameraProperties = new SimCameraProperties();

    public final CameraConfiguration cameraConfiguration;

    public SimulatedCameraConfiguration(CameraConfiguration cameraConfiguration) {
        this.cameraConfiguration = cameraConfiguration;

        double fovHorizontalRads = Units.degreesToRadians(this.cameraConfiguration.cameraFOV.horizontalDegrees / 2);
        double fovVerticalRads = Units.degreesToRadians(this.cameraConfiguration.cameraFOV.verticalDegrees / 2);

        double fovDiagonalRads = 2 * Math.atan(Math.hypot(Math.tan(fovHorizontalRads), Math.tan(fovVerticalRads)));

        Rotation2d fovDiagonal = Rotation2d.fromRadians(fovDiagonalRads);

        withSpecs(cameraConfiguration.cameraResolution.widthPixels, cameraConfiguration.cameraResolution.heightPixels, fovDiagonal);
    }

    /**
     * Configure the simulated camera to have the given specifications. Called automatically by constructor.
     * @param xPixels Resolution width of the camera
     * @param yPixels Resolution height of the camera
     * @param diagFOV Diagonal FOV of the camera
     * @return this {@link SimulatedCameraConfiguration} for chaining
     */
    public SimulatedCameraConfiguration withSpecs(int xPixels, int yPixels, Rotation2d diagFOV) {
        photonSimCameraProperties.setCalibration(xPixels, yPixels, diagFOV);
        return this;
    }
}
