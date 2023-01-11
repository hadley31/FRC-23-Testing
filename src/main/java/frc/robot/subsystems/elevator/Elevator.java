package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final static double kMinHeightMeters = Units.feetToMeters(1.5);
    private final static double kMaxHeightMeters = Units.feetToMeters(6.0);

    private ElevatorIO m_io;
    private ElevatorIOInputsAutoLogged m_inputs;
    private SlewRateLimiter m_rateLimiter;
    private double m_heightSetpointMeters;

    public Elevator(ElevatorIO io) {
        m_io = io;
        m_inputs = new ElevatorIOInputsAutoLogged();
        m_rateLimiter = new SlewRateLimiter(0.1);
    }

    @Override
    public void periodic() {
        double height = m_rateLimiter.calculate(m_heightSetpointMeters);

        m_io.setHeightMeters(height);

        m_io.updateInputs(m_inputs);
    }

    public void setHeightSetpoint(double height) {
        m_heightSetpointMeters = clampHeight(height);
    }

    public double getHeightMeters() {
        return m_io.getHeightMeters();
    }

    public void raise(double amount) {
        setHeightSetpoint(m_heightSetpointMeters + amount);
    }

    public void lower(double amount) {
        setHeightSetpoint(m_heightSetpointMeters - amount);
    }

    private double calculatePercentHeight(double heightMeters) {
        return (heightMeters - kMinHeightMeters) / (kMaxHeightMeters - kMinHeightMeters);
    }

    private double clampHeight(double heightMeters) {
        if (heightMeters < kMinHeightMeters || heightMeters >= kMaxHeightMeters) {
            DriverStation.reportWarning("Attempting to set invalid elevator position: " + heightMeters, false);
            return MathUtil.clamp(heightMeters, kMinHeightMeters, kMaxHeightMeters);
        }

        return heightMeters;
    }

    public CommandBase raiseCommand(double amount) {
        return runOnce(() -> raise(amount));
    }

    public CommandBase lowerCommand(double amount) {
        return runOnce(() -> lower(amount));
    }

    public CommandBase setPositionCommand(double heightMeters) {
        return runOnce(() -> setHeightSetpoint(heightMeters));
    }
}
