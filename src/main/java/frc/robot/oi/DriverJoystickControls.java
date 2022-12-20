package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverJoystickControls implements DriverControls {
    private final CommandJoystick m_leftJoystick;
    private final CommandJoystick m_rightJoystick;

    public DriverJoystickControls(int leftPort, int rightPort) {
        m_leftJoystick = new CommandJoystick(leftPort);
        m_rightJoystick = new CommandJoystick(rightPort);
    }

    @Override
    public double getLeftInputX() {
        return m_leftJoystick.getX();
    }

    @Override
    public double getLeftInputY() {
        return m_leftJoystick.getY();
    }

    @Override
    public double getRightInputX() {
        return m_rightJoystick.getX();
    }

    @Override
    public double getRightInputY() {
        return m_rightJoystick.getY();
    }

    @Override
    public Trigger getRobotRelativeDriveMode() {
        return m_rightJoystick.button(2);
    }

    @Override
    public Trigger getTargetAngleJoystickDriveMode() {
        return m_leftJoystick.button(4);
    }

    @Override
    public void setRumble(double value) {
        m_leftJoystick.getHID().setRumble(RumbleType.kLeftRumble, value);
        m_leftJoystick.getHID().setRumble(RumbleType.kRightRumble, value);
        m_rightJoystick.getHID().setRumble(RumbleType.kLeftRumble, value);
        m_rightJoystick.getHID().setRumble(RumbleType.kRightRumble, value);
    }
}
