package frc.robot.commands.drive.driver;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveCommandWrapper extends BaseDriveCommand {
    private final BaseDriveCommand m_command;

    public DriveCommandWrapper(BaseDriveCommand driveCommand) {
        super(driveCommand.m_drive);
        m_command = driveCommand;
    }

    /**
     * Override Built-In Methods
     */

    @Override
    public void initialize() {
        m_command.initialize();
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_command.getRequirements();
    }

    /**
     * Override BaseDriveCommand Methods
     */

    @Override
    protected double getXSpeed() {
        return m_command.getXSpeed();
    }

    @Override
    protected double getYSpeed() {
        return m_command.getYSpeed();
    }

    @Override
    protected double getRotationSpeed() {
        return m_command.getRotationSpeed();
    }

    @Override
    protected boolean getFieldRelative() {
        return m_command.getFieldRelative();
    }
}
