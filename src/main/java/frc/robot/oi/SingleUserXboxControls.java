package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleUserXboxControls implements DriverControls, OperatorControls {

    private final CommandXboxController m_controller;

    public SingleUserXboxControls(int port) {
        m_controller = new CommandXboxController(port);
    }

    @Override
    public Trigger getExampleControl() {
        return m_controller.a();
    }

    @Override
    public double getLeftInputX() {
        return m_controller.getLeftX();
    }

    @Override
    public double getLeftInputY() {
        return m_controller.getLeftY();
    }

    @Override
    public double getRightInputX() {
        return m_controller.getRightX();
    }

    @Override
    public double getRightInputY() {
        return m_controller.getRightY();
    }

    @Override
    public Trigger getRobotRelativeDriveMode() {
        return m_controller.rightBumper();
    }

    @Override
    public Trigger getTargetAngleJoystickDriveMode() {
        return new Trigger(() -> m_controller.getLeftTriggerAxis() > 0.2);
    }
}
