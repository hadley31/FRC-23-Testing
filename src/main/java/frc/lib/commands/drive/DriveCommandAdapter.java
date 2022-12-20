package frc.lib.commands.drive;

import frc.robot.subsystems.drive.Drive;

/**
 * Provides default implementation for {@link BaseDriveCommand} abstract methods
 */
public class DriveCommandAdapter extends BaseDriveCommand {

    public DriveCommandAdapter(Drive drive) {
        super(drive);
    }

    @Override
    protected double getXSpeed() {
        return 0;
    }

    @Override
    protected double getYSpeed() {
        return 0;
    }

    @Override
    protected double getRotationSpeed() {
        return 0;
    }

    @Override
    protected boolean getFieldRelative() {
        return false;
    }

}
