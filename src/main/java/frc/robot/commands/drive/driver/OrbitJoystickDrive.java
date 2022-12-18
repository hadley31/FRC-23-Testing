package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class OrbitJoystickDrive extends FaceTargetDrive {

    public OrbitJoystickDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier,
            Supplier<Translation2d> targetSupplier) {
        super(drive, xInputSupplier, yInputSupplier, targetSupplier);
    }

    public OrbitJoystickDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier,
            Translation2d target) {
        super(drive, xInputSupplier, yInputSupplier, target);
    }

    @Override
    protected boolean getFieldRelative() {
        return false;
    }
}
