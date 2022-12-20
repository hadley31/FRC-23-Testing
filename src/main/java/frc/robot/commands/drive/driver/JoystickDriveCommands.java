package frc.robot.commands.drive.driver;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.FaceTargetDrive;
import frc.robot.oi.DriverControls;
import frc.robot.subsystems.drive.Drive;

public class JoystickDriveCommands {
    public static BaseDriveCommand normal(Drive drive, DriverControls controls) {
        return new JoystickDrive(drive, controls::getLeftInputY, controls::getLeftInputX, controls::getRightInputX);
    }

    public static BaseDriveCommand faceTarget(Drive drive, DriverControls controls, Pose2d target) {
        return new FaceTargetDrive(drive, target)
                .withLinearSpeedSuppliers(controls::getLeftInputY, controls::getLeftInputX);
    }
}
