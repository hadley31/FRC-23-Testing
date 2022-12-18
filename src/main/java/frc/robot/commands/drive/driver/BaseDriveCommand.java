// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.driver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

/** This command class is highly adapted from 2363's swerve drive commands */
public abstract class BaseDriveCommand extends CommandBase {
    protected final Drive m_drive;

    public BaseDriveCommand(Drive drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public final void execute() {
        double xSpeed = getXSpeed();
        double ySpeed = getYSpeed();
        double rotationSpeed = getRotationSpeed();
        boolean isFieldRelative = getFieldRelative();

        var speeds = isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_drive.getPose().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        m_drive.drive(speeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public final boolean runsWhenDisabled() {
        return false;
    }

    protected abstract double getXSpeed();

    protected abstract double getYSpeed();

    protected abstract double getRotationSpeed();

    protected abstract boolean getFieldRelative();

    public BaseDriveCommand withFieldRelative(boolean enabled) {
        return new DriveCommandWrapper(this) {
            @Override
            protected boolean getFieldRelative() {
                return enabled;
            }
        };
    }
}
