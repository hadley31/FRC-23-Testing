package frc.robot.subsystems.drive.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIO;

public class CompetitionChassis implements DriveChassis {
    // Chassis Measurements
    public static final double kTrackWdith = Units.inchesToMeters(19.75);
    public static final double kWheelBase = Units.inchesToMeters(19.75);

    public static final Translation2d[] kSwerveModuleTranslations = {
            new Translation2d(kTrackWdith / 2.0, kWheelBase / 2.0), //values for front left (+, +)
            new Translation2d(kTrackWdith / 2.0, -kWheelBase / 2.0), //values for front right (+, -)
            new Translation2d(-kTrackWdith / 2.0, kWheelBase / 2.0), //values for back left (-, +)
            new Translation2d(-kTrackWdith / 2.0, -kWheelBase / 2.0) //values for back right (-, -)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kSwerveModuleTranslations);

    private final SwerveModuleIO[] m_modules;
    private final GyroIO m_gyro;

    public CompetitionChassis(SwerveModuleIO[] modules, GyroIO gyro) {
        assert modules.length == 4;
        m_modules = modules;
        m_gyro = gyro;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kDriveKinematics;
    }

    @Override
    public Translation2d[] getModuleTranslations() {
        return kSwerveModuleTranslations;
    }

    @Override
    public SwerveModuleIO[] getModules() {
        return m_modules;
    }

    @Override
    public GyroIO getGyro() {
        return m_gyro;
    }

    @Override
    public double getTrackWidth() {
        return kTrackWdith;
    }

    @Override
    public double getWheelBase() {
        return kWheelBase;
    }
}
