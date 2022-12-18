package frc.robot.commands.drive.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class LinearJoystickDrive extends BaseDriveCommand {
    protected final Supplier<Double> m_xInputSupplier;
    protected final Supplier<Double> m_yInputSupplier;

    protected final JoystickAxis m_xInputAxis = new JoystickAxis(0.2);
    protected final JoystickAxis m_yInputAxis = new JoystickAxis(0.2);

    public LinearJoystickDrive(Drive drive, Supplier<Double> xInputSupplier, Supplier<Double> yInputSupplier) {
        super(drive);
        m_xInputSupplier = xInputSupplier;
        m_yInputSupplier = yInputSupplier;
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
        return 0;
    }

    @Override
    protected boolean getFieldRelative() {
        return true;
    }

    public static class JoystickAxis {
        private final double m_deadzone;

        public JoystickAxis(double deadzone) {
            m_deadzone = Math.abs(deadzone);
        }

        public double calculate(double input) {
            return MathUtil.applyDeadband(input, m_deadzone);
        }
    }
}
