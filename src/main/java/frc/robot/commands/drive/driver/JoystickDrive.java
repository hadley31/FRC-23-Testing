package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import frc.lib.commands.drive.BaseDriveCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.oi.AxisInput;
import frc.robot.subsystems.drive.Drive;

public class JoystickDrive extends BaseDriveCommand {
    protected final Supplier<Double> m_xInputSupplier;
    protected final Supplier<Double> m_yInputSupplier;
    protected final Supplier<Double> m_thetaInputSupplier;

    protected final AxisInput m_xInputAxis = new AxisInput(0.13);
    protected final AxisInput m_yInputAxis = new AxisInput(0.13);
    protected final AxisInput m_thetaInputAxis = new AxisInput(0.13);

    public JoystickDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier,
            Supplier<Double> thetaInputSupplier) {
        super(drive);
        m_xInputSupplier = xInputSupplier;
        m_yInputSupplier = yInputSupplier;
        m_thetaInputSupplier = thetaInputSupplier;
    }

    @Override
    protected double getXSpeed() {
        return m_xInputAxis.calculate(m_xInputSupplier.get()) * DriveConstants.kMaxSpeedMetersPerSecond;
    }

    @Override
    protected double getYSpeed() {
        return m_yInputAxis.calculate(m_yInputSupplier.get()) * DriveConstants.kMaxSpeedMetersPerSecond;
    }

    @Override
    protected double getRotationSpeed() {
        return m_thetaInputAxis.calculate(m_thetaInputSupplier.get()) * DriveConstants.kMaxSpeedRadiansPerSecond;
    }

    @Override
    protected boolean getFieldRelative() {
        return true;
    }
}
