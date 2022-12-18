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
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getLeftInputX() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getLeftInputY() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightInputX() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightInputY() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Trigger getRobotRelativeDriveMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Trigger getTargetFollowDriveMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Trigger getTargetAngleJoystickDriveMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Trigger getOrbitDriveMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Trigger getTest1Button() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Trigger getRumbleButton() {
        // TODO Auto-generated method stub
        return null;
    }

}
