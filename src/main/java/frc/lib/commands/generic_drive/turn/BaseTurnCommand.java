package frc.lib.commands.generic_drive.turn;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.commands.generic_drive.DriveCommandConfig;
import frc.lib.commands.generic_drive.TargetAngleDrive;

public abstract class BaseTurnCommand extends TargetAngleDrive {

    public BaseTurnCommand(DriveCommandConfig drive, PIDController controller) {
        super(drive, controller);
    }

    @Override
    protected final double getXSpeed() {
        return 0;
    }

    @Override
    protected final double getYSpeed() {
        return 0;
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
