package frc.robot.subsystems.drive.modules;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIO.SwerveModuleInputs;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleInputs> {

    @AutoLog
    public static class SwerveModuleInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        // public double driveVelocityFilteredMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        // public double[] driveCurrentAmps = new double[] {};
        // public double[] driveTempCelcius = new double[] {};

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        // public double[] turnCurrentAmps = new double[] {};
        // public double[] turnTempCelcius = new double[] {};
    }

    public Rotation2d getRotation();

    public Rotation2d getAbsoluteRotation();

    public double getVelocityMetersPerSecond();

    public double getDrivePositionMeters();

    public void zeroDriveEncoder();

    public default SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getRotation());
    }

    public default SwerveModuleState getAbsoluteState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getAbsoluteRotation());
    }

    public default SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getRotation());
    }

    public default SwerveModulePosition getAbsolutePosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getAbsoluteRotation());
    }

    public void setDesiredState(SwerveModuleState state);

    public void zeroTurnEncoder();

    public void syncTurnEncoderWithAbsolute();

    public default void setTurnPID(double p, double i, double d) {
    }

    public default void setDrivePID(double p, double i, double d) {
    }

    /**
     * Used primarily for moving the robot while disabled
     * @param coast
     */
    public default void setTurnBrakeMode(boolean coast) {
    }

    /**
     * Used during autonomous path following or to quickly stop the robot
     * @param brake Whether brake mode should be enabled
     */
    public default void setDriveBrakeMode(boolean brake) {
    }
}
