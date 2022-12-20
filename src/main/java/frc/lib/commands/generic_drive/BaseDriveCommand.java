// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands.generic_drive;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * An abstract command which provides base logic for driving a swerve base.
 * Inherit from this class to provide functionality on how the swerve drive should move
 * in the x, y, and rotational axes, and whether it should move relative to the robot (robot relative)
 * or relative to the driver (field relative).
 * This command class is highly adapted from 2363's swerve drive commands
 */
public abstract class BaseDriveCommand extends CommandBase {
    protected final DriveCommandConfig m_drive;

    public BaseDriveCommand(DriveCommandConfig config) {
        m_drive = config;
        addRequirements(config.getRequirements());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public final void execute() {
        double x = getXSpeed();
        double y = getYSpeed();
        double omega = getRotationSpeed();
        boolean isFieldRelative = getFieldRelative();

        ChassisSpeeds speeds = getChassisSpeeds(x, y, omega, isFieldRelative);
        m_drive.acceptSpeeds(speeds);
    }

    /**
    * Combines all returned movement values to create output chassis ChassisSpeeds
    * Putting this logic in its own method allows {@link DriveCommandWrapper}s to
    * modify the late stage chassis speeds being sent to the drive command
    * Do <strong>NOT</strong> override this method unless you know what you are doing!
    * @return the desired robot chassis speeds
    */
    protected ChassisSpeeds getChassisSpeeds(double x, double y, double omega, boolean isFieldRelative) {
        return isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_drive.getPose().getRotation())
                : new ChassisSpeeds(x, y, omega);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.acceptSpeeds(new ChassisSpeeds());
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

    /**
     * @return the x (forward positive) speed in meters per second
     */
    protected abstract double getXSpeed();

    /**
     * @return the y (left positive) speed in meters per second 
     */
    protected abstract double getYSpeed();

    /**
     * @return the angular (CCW positive) speed in radians per second
     */
    protected abstract double getRotationSpeed();

    /**
     * Determines whether robot speeds are relative to the driver (field relative)
     * or relative to the robot (robot relative)
     * @return true if field relative, false if robot relative
     */
    protected abstract boolean getFieldRelative();

    /**
     * @param xSpeedSupplier provides the x speed input [-1, 1]
     * @param ySpeedSupplier provides the y speed input [-1, 1]
     * @return a wrapped drive command which uses the specified linear movement input suppliers
     */
    public final BaseDriveCommand withLinearSpeedSuppliers(Supplier<Double> xSpeedSupplier,
            Supplier<Double> ySpeedSupplier) {
        return new DriveCommandWrapper(this) {
            @Override
            protected double getXSpeed() {
                return xSpeedSupplier.get();
            }

            @Override
            protected double getYSpeed() {
                return ySpeedSupplier.get();
            }
        };
    }

    /**
    * @param xInputSupplier provides the x speed input [-1, 1]
    * @param yInputSupplier provides the y speed input [-1, 1]
    * @return a wrapped drive command which uses the specified angular movement input supplier
    */
    public final BaseDriveCommand withAngularSpeedSupplier(Supplier<Double> omegaSupplier) {
        return new DriveCommandWrapper(this) {
            @Override
            protected double getRotationSpeed() {
                return omegaSupplier.get();
            }
        };
    }

    /**
     * Returns a new wrapped {@link BaseDriveCommand} with field relative set to the desired value
     * @param enabled
     * @return
     */
    public final BaseDriveCommand withFieldRelative(boolean enabled) {
        return new DriveCommandWrapper(this) {
            @Override
            protected boolean getFieldRelative() {
                return enabled;
            }
        };
    }

    public static BaseDriveCommand create(DriveCommandConfig config) {
        return new DriveCommandAdapter(config);
    }
}
