package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;

public abstract class TargetAngleDrive extends LinearJoystickDrive {
    private final PIDController m_controller = new PIDController(10, 0.1, .02);

    public TargetAngleDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier) {
        super(drive, xInputSupplier, yInputSupplier);
        m_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    protected final double getRotationSpeed() {
        Rotation2d currentRotation = m_drive.getPose().getRotation();
        Rotation2d desiredRotation = getDesiredAngle();

        return m_controller.calculate(currentRotation.getRadians(), desiredRotation.getRadians());
    }

    /**
     * 
     * @param offset
     * @return a new instance of the TargetAngleDrive with a specified offset rotation
     */
    public final TargetAngleDrive withOffset(Rotation2d offset) {
        return new TargetAngleDriveOffsetWrapper(this, offset);
    }

    /**
     * To use FaceTargetDrive as an example, where the robot will face towards a given target,
     * this method will return an instance that faces directly opposite from the given target.
     * @return a new instance of the TargetAngleDrive with an offset of 180 degrees
     */
    public final TargetAngleDrive asInverted() {
        return withOffset(Rotation2d.fromDegrees(180));
    }

    /**
     * 
     * @return the target robot gyro angle
     */
    protected abstract Rotation2d getDesiredAngle();
}
