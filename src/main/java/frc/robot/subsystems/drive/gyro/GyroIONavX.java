package frc.robot.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.lib.advantagekit.LoggedTunableNumber;

public class GyroIONavX extends GyroIOWPI {
  private final AHRS m_navx;

  public GyroIONavX(AHRS navx) {
    m_navx = navx;

    LoggedTunableNumber tunableYaw = new LoggedTunableNumber("Gyro/Yaw", 0);
    LoggedTunableNumber tunablePitch = new LoggedTunableNumber("Gyro/Pitch", 0);
    LoggedTunableNumber tunableRoll = new LoggedTunableNumber("Gyro/Roll", 0);

    tunableYaw.addListener(x -> setAngleOffset(Rotation2d.fromDegrees(x)));
    tunablePitch.addListener(x -> setPitchOffset(Rotation2d.fromDegrees(x)));
    tunableRoll.addListener(x -> setRollOffset(Rotation2d.fromDegrees(x)));
  }

  public GyroIONavX(Port port) {
    this(new AHRS(port));
  }

  @Override
  public AHRS getWPIGyro() {
    return m_navx;
  }

  @Override
  public Rotation2d getRawGyroPitch() {
    return Rotation2d.fromDegrees(m_navx.getPitch());
  }

  @Override
  public Rotation2d getRawGyroRoll() {
    return Rotation2d.fromDegrees(m_navx.getRoll());
  }
}
