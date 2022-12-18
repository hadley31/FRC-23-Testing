package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls extends RumbleControls {
    public double getLeftInputX();

    public double getLeftInputY();

    public double getRightInputX();

    public double getRightInputY();

    public Trigger getRobotRelativeDriveMode();

    public Trigger getTargetFollowDriveMode();

    public Trigger getTargetAngleJoystickDriveMode();

    public Trigger getOrbitDriveMode();

    public Trigger getTest1Button();

    public Trigger getRumbleButton();

    @Override
    public default void setRumble(double value) {
        System.out.println("Default setHaptics() ... Override me!");
    }
}
