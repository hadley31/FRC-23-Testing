package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import frc.robot.subsystems.drive.Drive;

public class RobotRelativeJoystickDrive extends JoystickDrive {

    public RobotRelativeJoystickDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier,
            Supplier<Double> thetaInputSupplier) {
        super(drive, xInputSupplier, yInputSupplier, thetaInputSupplier);
    }

    @Override
    protected boolean getFieldRelative() {
        return false;
    }
}
