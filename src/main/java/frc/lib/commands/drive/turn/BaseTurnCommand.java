package frc.lib.commands.drive.turn;

import frc.lib.commands.drive.TargetAngleDrive;
import frc.robot.subsystems.drive.Drive;

public abstract class BaseTurnCommand extends TargetAngleDrive {

    public BaseTurnCommand(Drive drive) {
        super(drive);
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

    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
    }
}
