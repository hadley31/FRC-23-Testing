package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.chassis.DriveChassis;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.drive.modules.SwerveModuleIO;
import frc.robot.subsystems.drive.modules.SwerveModuleInputsAutoLogged;

public class Drive extends SubsystemBase {

    private final DriveChassis m_chassis;

    private final SwerveModuleInputsAutoLogged[] m_moduleInputs = {
            new SwerveModuleInputsAutoLogged(),
            new SwerveModuleInputsAutoLogged(),
            new SwerveModuleInputsAutoLogged(),
            new SwerveModuleInputsAutoLogged(),
    };

    private final GyroInputsAutoLogged m_gyroInputs = new GyroInputsAutoLogged();

    public Drive(DriveChassis chassis) {
        m_chassis = chassis;
    }

    @Override
    public void periodic() {
        // Update Gyro Log Inputs
        getChassis().getGyro().updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", m_gyroInputs);

        // Update Swerve Module Log Inputs
        var modules = getChassis().getModules();
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(m_moduleInputs[i]);

            String key = "Drive/Module" + i;
            Logger.getInstance().processInputs(key, m_moduleInputs[i]);
        }

        // Log robot position and states
        Logger.getInstance().recordOutput("ModuleStates", getModuleStates());
    }

    public DriveChassis getChassis() {
        return m_chassis;
    }

    public void drive(ChassisSpeeds speeds) {
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            brake();
            return;
        }

        Logger.getInstance().recordOutput(
                "DesiredSpeeds",
                new double[] {
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond
                });

        SwerveModuleState[] swerveModuleStates = getChassis().getKinematics().toSwerveModuleStates(speeds);

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState... desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                DriveConstants.kMaxSpeedMetersPerSecond);

        Logger.getInstance().recordOutput("DesiredModuleStates", desiredStates);

        getChassis().setDesiredModuleStates(desiredStates);
    }

    public SwerveModuleState[] getModuleStates() {
        return getChassis().getModuleStates();
    }

    public SwerveModulePosition[] getModulePositions() {
        return getChassis().getModulePositions();
    }

    public void brake() {
        for (SwerveModuleIO module : getChassis().getModules()) {
            module.setDesiredState(new SwerveModuleState(0, module.getRotation()));
        }
    }

    public void setTurnCoastMode(boolean coast) {
        getChassis().setTurnCoastMode(coast);
    }

    public void setDriveBrakeMode(boolean brake) {
        getChassis().setDriveBrakeMode(brake);
    }

    public Rotation2d getHeading() {
        return getChassis().getGyroAngle();
    }

    //#region Sim Stuff

    private double m_simGyroLastUpdated;

    @Override
    public void simulationPeriodic() {
        double gyroDelta = getChassis().getKinematics().toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
        double ts = Timer.getFPGATimestamp();

        double deltaTime = ts - m_simGyroLastUpdated;

        getChassis().getGyro().addAngle(Rotation2d.fromRadians(gyroDelta * deltaTime));
        m_simGyroLastUpdated = ts;
    }

    //#endregion

    //#region Commands

    public CommandBase brakeCommand() {
        return runOnce(this::brake);
    }

    public CommandBase setBrakeModeCommand(boolean driveEnabled, boolean turnEnabled) {
        return runOnce(() -> {
            setDriveBrakeMode(driveEnabled);
            setTurnCoastMode(!turnEnabled);
        });
    }

    public CommandBase setBrakeModeCommand(boolean enabled) {
        return setBrakeModeCommand(enabled, enabled);
    }

    //#endregion
}
