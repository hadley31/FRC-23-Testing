package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;

public abstract class TargetAngleSupplierDrive extends TargetAngleDrive {
    private final Supplier<Rotation2d> m_rotationSupplier;

    public TargetAngleSupplierDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier,
            Supplier<Rotation2d> rotationSupplier) {
        super(drive, xInputSupplier, yInputSupplier);
        m_rotationSupplier = rotationSupplier;
    }

    @Override
    protected Rotation2d getDesiredAngle() {
        return m_rotationSupplier.get();
    }
}
