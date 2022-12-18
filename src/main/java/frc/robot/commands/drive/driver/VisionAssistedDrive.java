package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;

public class VisionAssistedDrive extends JoystickDrive {

    private static final double kAngleThreshold = 3.0;
    private static final double kCameraAssistedSpeedMultiplier = 2.0;
    private static final double kMinimumCameraAssistedSpeed = 0.1;

    private final Camera m_camera;
    private final int m_pipelineIndex;

    public VisionAssistedDrive(Drive drive, Camera camera, int pipelineIndex, Supplier<Double> xInputSupplier,
            Supplier<Double> yInputSupplier,
            Supplier<Double> thetaInputSupplier) {
        super(drive, xInputSupplier, yInputSupplier, thetaInputSupplier);
        m_camera = camera;
        m_pipelineIndex = pipelineIndex;
        addRequirements(m_camera);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_camera.setPipelineIndex(m_pipelineIndex, true);
    }

    @Override
    protected double getRotationSpeed() {
        double thetaInput = super.getRotationSpeed();

        if (thetaInput != 0.0) {
            return thetaInput;
        }

        return getCameraInputSpeed();
    }

    private double getCameraInputSpeed() {
        var result = m_camera.getLatestResult();

        if (!result.hasTargets()) {
            return 0.0;
        }

        var bestTarget = result.getBestTarget();
        double targetYawOffset = bestTarget.getYaw();

        if (Math.abs(targetYawOffset) < kAngleThreshold) {
            return 0.0;
        }

        double m = kCameraAssistedSpeedMultiplier / VisionConstants.kCameraYawFOV;
        double b = Math.copySign(kMinimumCameraAssistedSpeed, targetYawOffset);

        return m * targetYawOffset + b;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_camera.setLEDs(false);
    }
}
