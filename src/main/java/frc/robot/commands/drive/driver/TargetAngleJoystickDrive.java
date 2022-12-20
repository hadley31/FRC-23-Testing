package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.commands.drive.TargetAngleDrive;
import frc.robot.subsystems.drive.Drive;

public class TargetAngleJoystickDrive extends TargetAngleDrive {

    private final Supplier<Double> m_rotationXInputSupplier;
    private final Supplier<Double> m_rotationYInputSupplier;
    private Rotation2d m_desiredRotation;

    public TargetAngleJoystickDrive(Drive drive,
            Supplier<Double> rotationXInputSupplier, Supplier<Double> rotationYInputSupplier) {
        super(drive);
        m_rotationXInputSupplier = rotationXInputSupplier;
        m_rotationYInputSupplier = rotationYInputSupplier;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_desiredRotation = m_drive.getPose().getRotation();
    }

    @Override
    protected Rotation2d getDesiredAngle() {
        double inputX = m_rotationXInputSupplier.get();
        double inputY = m_rotationYInputSupplier.get();

        if (Math.hypot(inputX, inputY) < 0.7) {
            return m_desiredRotation;
        }

        m_desiredRotation = new Rotation2d(inputX, inputY);

        return m_desiredRotation;
    }

}
