package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;

public class ChargeStationBalance extends CommandBase {
    private double root2Over2 = Math.sqrt(2) / 2;
    private final Drive m_drive;

    public ChargeStationBalance(Drive drive) {
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.setDriveBrakeMode(true);
        m_drive.setTurnCoastMode(false);
    }

    @Override
    public void execute() {
        GyroIO gyro = m_drive.getChassis().getGyro();
        // TODO: utilize both pitch and roll
        // boolean usePitch = Math.abs(gyro.getRotation2d().getCos()) < root2Over2;
        // double angle = usePitch ? gyro.getPitch() : gyro.getRoll();

        ChassisSpeeds speeds = null;
        double angle = gyro.getPitch();

        if (Math.abs(angle) < Units.degreesToRadians(10)) {
            speeds = new ChassisSpeeds();
        } else {
            double x = angle;
            speeds = new ChassisSpeeds(x, 0, 0);
        }

        m_drive.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
    }
}
