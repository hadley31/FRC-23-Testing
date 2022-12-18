package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverJoystickControls implements DriverControls {

    private static final Trigger kEmptyTrigger = new Trigger(() -> false);

    private final Joystick m_leftJoystick;
    private final Joystick m_rightJoystick;

    public DriverJoystickControls(int leftPort, int rightPort) {
        m_leftJoystick = new Joystick(leftPort);
        m_rightJoystick = new Joystick(rightPort);
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
        return new Trigger(() -> m_rightJoystick.getRawButton(2));
    }

    @Override
    public Trigger getTargetFollowDriveMode() {
        return new Trigger(() -> m_leftJoystick.getRawButton(2));
    }

    @Override
    public Trigger getTargetAngleJoystickDriveMode() {
        return new Trigger(() -> m_leftJoystick.getRawButton(4));
    }

    @Override
    public Trigger getOrbitDriveMode() {
        return kEmptyTrigger;
    }

    @Override
    public Trigger getTest1Button() {
        // TODO Auto-generated method stub
        return kEmptyTrigger;
    }

    @Override
    public Trigger getRumbleButton() {
        // TODO Auto-generated method stub
        return kEmptyTrigger;
    }

    @Override
    public void setRumble(double value) {
        m_leftJoystick.setRumble(RumbleType.kLeftRumble, value);
        m_leftJoystick.setRumble(RumbleType.kRightRumble, value);
        m_rightJoystick.setRumble(RumbleType.kLeftRumble, value);
        m_rightJoystick.setRumble(RumbleType.kRightRumble, value);
    }
}
