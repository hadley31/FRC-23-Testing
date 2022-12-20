package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverXboxControls implements DriverControls {
    private final CommandXboxController m_controller;

    public DriverXboxControls(int port) {
        m_controller = new CommandXboxController(port);
    }

    @Override
    public double getLeftInputX() {
        return -m_controller.getLeftX();
    }

    @Override
    public double getLeftInputY() {
        return -m_controller.getLeftY();
    }

    @Override
    public double getRightInputX() {
        return -m_controller.getRightX();
    }

    @Override
    public double getRightInputY() {
        return -m_controller.getRightY();
    }

    @Override
    public Trigger getRobotRelativeDriveMode() {
        return new Trigger(() -> m_controller.getRightTriggerAxis() > 0.5);
    }

    @Override
    public Trigger getTargetAngleJoystickDriveMode() {
        return new Trigger(() -> m_controller.getLeftTriggerAxis() > 0.5);
    }

    @Override
    public void setRumble(double value) {
        System.out.println("RUMBLING: " + value);
        m_controller.getHID().setRumble(RumbleType.kLeftRumble, value);
        m_controller.getHID().setRumble(RumbleType.kRightRumble, value);
    }
}
