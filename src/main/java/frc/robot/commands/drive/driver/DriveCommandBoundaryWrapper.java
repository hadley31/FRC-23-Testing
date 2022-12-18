package frc.robot.commands.drive.driver;

import frc.robot.Constants.FieldConstants;

public class DriveCommandBoundaryWrapper extends DriveCommandWrapper {

    public DriveCommandBoundaryWrapper(BaseDriveCommand driveCommand) {
        super(driveCommand);
    }

    @Override
    protected double getXSpeed() {
        double x = super.getXSpeed();
        if (x <= 0 && m_drive.getPose().getX() <= 0) {
            return 0;
        }
        if (x >= 0 && m_drive.getPose().getX() >= FieldConstants.kFieldWidthMeters) {
            return 0;
        }

        return x;
    }

    @Override
    protected double getYSpeed() {
        double y = super.getYSpeed();
        if (y <= 0 && m_drive.getPose().getY() <= 0) {
            return 0;
        }
        if (y >= 0 && m_drive.getPose().getY() >= FieldConstants.kFieldHeightMeters) {
            return 0;
        }

        return y;
    }

}
