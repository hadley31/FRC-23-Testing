package frc.robot.subsystems.vision;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.photonvision.PhotonCamera;
import frc.robot.Constants.VisionConstants;

public class Camera extends SubsystemBase {

    private PhotonCamera m_camera;
    private PhotonPipelineResult m_latestResult;

    public Camera(PhotonCamera camera) {
        m_camera = camera;
    }

    @Override
    public void periodic() {
        m_latestResult = m_camera.getLatestResult();
    }

    /**
     * Sets the camera's image processing pipeline
     * @param index The desired pipeline index
     */
    public void setPipelineIndex(int index) {
        m_camera.setPipelineIndex(index);
    }

    /**
     * Sets the camera's image processing pipeline
     * @param index The desired pipeline index
     * @param autoConfigureLEDs Whether the camera should auto-detect if the pipeline requires LEDs and enable them
     */
    public void setPipelineIndex(int index, boolean autoConfigureLEDs) {
        m_camera.setPipelineIndex(index);

        if (autoConfigureLEDs) {
            boolean pipelineRequiresLEDs = VisionConstants.kPipelinesRequiringLEDs.contains(index);
            setLEDs(pipelineRequiresLEDs);
        }
    }

    public void setLEDs(boolean enabled) {
        m_camera.setLED(enabled ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    public boolean getLEDState() {
        return m_camera.getLEDMode() == VisionLEDMode.kOn;
    }

    public PhotonPipelineResult getLatestResult() {
        return m_latestResult;
    }

    public PhotonCamera getPhotonCamera() {
        return m_camera;
    }

    //#region Commands

    public CommandBase setPipelineIndexCommand(int index) {
        return runOnce(() -> setPipelineIndex(index, true));
    }

    public CommandBase setLEDStateComand(boolean enabled) {
        return runOnce(() -> setLEDs(enabled));
    }

    public CommandBase toggleLEDStateCommand() {
        return runOnce(() -> setLEDs(!getLEDState()));
    }

    //#endregion
}