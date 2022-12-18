package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorXboxControls implements OperatorControls {

    private final CommandXboxController m_controller;

    public OperatorXboxControls(int port) {
        m_controller = new CommandXboxController(port);
    }

    @Override
    public Trigger getExampleControl() {
        return m_controller.a();
    }
}
