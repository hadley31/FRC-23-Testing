package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;

public class PigeonIO extends GyroIOWPI {
    private WPI_Pigeon2 m_pigeon;

    public PigeonIO(WPI_Pigeon2 pigeon) {
        m_pigeon = pigeon;

        m_pigeon.configMountPose(AxisDirection.PositiveX, AxisDirection.NegativeZ);
    }

    public PigeonIO(int port) {
        this(new WPI_Pigeon2(port));
    }

    @Override
    public WPI_Pigeon2 getWPIGyro() {
        return m_pigeon;
    }

    @Override
    public double getPitch() {
        return Units.degreesToRadians(m_pigeon.getPitch());
    }

    @Override
    public double getRoll() {
        return Units.degreesToRadians(m_pigeon.getRoll());
    }
}
