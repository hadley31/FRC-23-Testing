package frc.robot.subsystems.drive.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIO;

public interface DriveChassis {
    public double getTrackWidth();

    public double getWheelBase();

    public Translation2d[] getModuleTranslations();

    public SwerveDriveKinematics getKinematics();

    public SwerveModuleIO[] getModules();

    public GyroIO getGyro();

    public default SwerveModuleState[] getModuleStates() {
        var swerveModules = getModules();
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }

        return states;
    }

    public default SwerveModulePosition[] getModulePositions() {
        var swerveModules = getModules();
        SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getPosition();
        }

        return states;
    }

    public default ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public default Rotation2d getGyroAngle() {
        return getGyro().getRotation2d();
    }

    public default void setTurnCoastMode(boolean coast) {
        for (SwerveModuleIO module : getModules()) {
            module.setTurnCoastMode(coast);
        }
    }

    public default void setDriveBrakeMode(boolean brake) {
        for (SwerveModuleIO module : getModules()) {
            module.setDriveBrakeMode(brake);
        }
    }

    public default void setDesiredModuleStates(SwerveModuleState... desiredStates) {
        var modules = getModules();
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }
}
