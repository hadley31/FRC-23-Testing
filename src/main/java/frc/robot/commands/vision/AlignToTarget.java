package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;

public class AlignToTarget extends CommandBase {
    private final Drive m_drive;
    private final Camera m_camera;
    private final int m_pipelineIndex;

    public AlignToTarget(Drive drive, Camera camera, int pipelineIndex) {
        m_drive = drive;
        m_camera = camera;
        m_pipelineIndex = pipelineIndex;

        addRequirements(drive, camera);
    }

    @Override
    public void initialize() {
        m_camera.setPipelineIndex(m_pipelineIndex, true);
        // m_drive.setDriveBrakeMode(true);
    }

    @Override
    public void execute() {
        var result = m_camera.getLatestResult();

        if (!result.hasTargets()) {
            m_drive.brake();
        }

        // var bestTarget = result.getBestTarget();
        // double targetYawOffset = bestTarget.getYaw();

        // if (Math.abs(targetYawOffset) < kAngleThreshold) {
        //     return 0.0;
        // }

        // double m = kCameraAssistedSpeedMultiplier / VisionConstants.kCameraMaxYaw;
        // double b = Math.copySign(kMinimumCameraAssistedSpeed, targetYawOffset);

        // return m * targetYawOffset + b;
    }

    @Override
    public void end(boolean interrupted) {
        m_camera.setLEDs(false);
        // m_drive.setDriveBrakeMode(false);
    }
}
