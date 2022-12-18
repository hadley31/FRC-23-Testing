package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class JoystickDrive extends LinearJoystickDrive {
    protected final Supplier<Double> m_thetaInputSupplier;
    protected final JoystickAxis m_thetaInputAxis = new JoystickAxis(0.2);

    public JoystickDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier,
            Supplier<Double> thetaInputSupplier) {
        super(drive, xInputSupplier, yInputSupplier);
        m_thetaInputSupplier = thetaInputSupplier;
    }

    @Override
    protected double getRotationSpeed() {
        return m_thetaInputAxis.calculate(m_thetaInputSupplier.get()) * DriveConstants.kMaxSpeedRadiansPerSecond;
    }
}
