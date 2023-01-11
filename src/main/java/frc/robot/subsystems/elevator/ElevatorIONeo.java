package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.lib.listeners.ChangeNotifier;

public class ElevatorIONeo implements ElevatorIO {
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_controller;
    private final ElevatorFeedforward m_feedforward;

    private final LoggedDashboardNumber m_p = new LoggedDashboardNumber("Elevator/P", 0.01);
    private final LoggedDashboardNumber m_i = new LoggedDashboardNumber("Elevator/I", 0);
    private final LoggedDashboardNumber m_d = new LoggedDashboardNumber("Elevator/D", 0.001);

    public ElevatorIONeo(int motorPort) {
        m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        m_feedforward = new ElevatorFeedforward(0.1, 0.1, 0.1);

        m_encoder = m_motor.getEncoder();
        m_controller = m_motor.getPIDController();

        m_controller.setP(0.01);
        m_controller.setI(0.00);
        m_controller.setD(0.001);

        ChangeNotifier.of(m_p::get).addListener(m_controller::setP);
        ChangeNotifier.of(m_i::get).addListener(m_controller::setI);
        ChangeNotifier.of(m_d::get).addListener(m_controller::setD);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorVoltage = m_motor.getBusVoltage();
    }

    @Override
    public void setHeightMeters(double heightMeters) {
        double ff = m_feedforward.calculate(m_encoder.getVelocity());
        m_controller.setReference(heightMeters, ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);
    }

    @Override
    public double getHeightMeters() {
        return m_encoder.getPosition();
    }
}
