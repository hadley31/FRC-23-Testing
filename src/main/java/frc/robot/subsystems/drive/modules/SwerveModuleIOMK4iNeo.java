package frc.robot.subsystems.drive.modules;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleIOMK4iNeo implements SwerveModuleIO {

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final RelativeEncoder m_driveMotorEncoder;
    private final RelativeEncoder m_turnMotorEncoder;

    private final WPI_CANCoder m_turnCANCoder;

    private final SparkMaxPIDController m_drivePIDController;
    private final SparkMaxPIDController m_turnPIDController;

    public SwerveModuleIOMK4iNeo(int turnMotorPort, int driveMotorPort, int turnCANCoderPort) {
        // Define Motors
        this.m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
        this.m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);

        // Set Idle Modes
        this.m_turnMotor.setIdleMode(IdleMode.kBrake);
        this.m_driveMotor.setIdleMode(IdleMode.kCoast);

        // Define Encoders
        m_turnMotorEncoder = m_turnMotor.getEncoder();
        m_driveMotorEncoder = m_driveMotor.getEncoder();

        m_turnCANCoder = new WPI_CANCoder(turnCANCoderPort);

        // Define Encoder Conversion Factors
        m_turnMotorEncoder.setPositionConversionFactor(DriveConstants.kTurnPositionConversionFactor); // radians
        m_turnMotorEncoder.setVelocityConversionFactor(DriveConstants.kTurnPositionConversionFactor / 60); // radians per second

        m_driveMotorEncoder.setPositionConversionFactor(DriveConstants.kDrivePositionConversionFactor); // meters
        m_driveMotorEncoder.setVelocityConversionFactor(DriveConstants.kDrivePositionConversionFactor / 60); // meters per second

        // Define PID Controllers
        m_turnPIDController = m_turnMotor.getPIDController();
        m_drivePIDController = m_driveMotor.getPIDController();

        setTurnPID(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
        setDrivePID(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput();
        inputs.drivePositionMeters = getDrivePositionMeters();
        inputs.driveVelocityMetersPerSec = getVelocityMetersPerSecond();

        inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput();
        inputs.turnAbsolutePositionRad = getAbsoluteRotation().getRadians();
        inputs.turnPositionRad = getRotation().getRadians();
        inputs.turnVelocityRadPerSec = m_turnMotorEncoder.getVelocity();
    }

    @Override
    public double getVelocityMetersPerSecond() {
        return m_driveMotorEncoder.getVelocity();
    }

    @Override
    public Rotation2d getRotation() {
        double angle = MathUtil.angleModulus(m_turnMotorEncoder.getPosition());
        return new Rotation2d(angle);
    }

    @Override
    public Rotation2d getAbsoluteRotation() {
        double angle = Units.degreesToRadians(m_turnCANCoder.getAbsolutePosition());
        return new Rotation2d(MathUtil.angleModulus(angle));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        /*
         * Here we are performing SwerveModuleState.optimize() manually
         * This is to allow the motor to turn continuously unbounded, while keeping our state bound to -pi to pi
         * SwerveModuleState.optimize() is only optimal on the range -pi to pi
         */
        double currentAngle = m_turnMotorEncoder.getPosition();

        double driveOutput = state.speedMetersPerSecond;

        double deltaAngle = MathUtil.angleModulus(state.angle.getRadians() - currentAngle);

        // Reverse drive direction - more efficient
        if (Math.abs(deltaAngle) > Constants.HALF_PI) {
            driveOutput *= -1;
            deltaAngle -= Math.copySign(Math.PI, deltaAngle);
        }

        // Set Turn Motor Setpoint
        double desiredAngle = currentAngle + deltaAngle;

        m_turnPIDController.setReference(desiredAngle, ControlType.kPosition);

        // Set Drive Motor Setpoint
        m_drivePIDController.setReference(driveOutput, ControlType.kVelocity, 0,
                DriveConstants.kDriveFF * driveOutput);
    }

    @Override
    public void zeroTurnEncoder() {
        m_turnMotorEncoder.setPosition(0.0);

        m_turnCANCoder.setPosition(0.0);

        double magnetOffset = m_turnCANCoder.configGetMagnetOffset() - m_turnCANCoder.getAbsolutePosition();
        m_turnCANCoder.configMagnetOffset(magnetOffset);
    }

    @Override
    public void syncTurnEncoderWithAbsolute() {
        m_turnMotorEncoder.setPosition(getAbsoluteRotation().getRadians());
    }

    @Override
    public double getDrivePositionMeters() {
        return m_driveMotorEncoder.getPosition();
    }

    @Override
    public void zeroDriveEncoder() {
        m_driveMotorEncoder.setPosition(0.0);
    }

    @Override
    public void setTurnBrakeMode(boolean brake) {
        m_turnMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setDriveBrakeMode(boolean brake) {
        m_driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnPID(double p, double i, double d) {
        m_turnPIDController.setP(p);
        m_turnPIDController.setI(i);
        m_turnPIDController.setD(d);
    }

    @Override
    public void setDrivePID(double p, double i, double d) {
        m_drivePIDController.setP(p);
        m_drivePIDController.setI(i);
        m_drivePIDController.setD(d);
    }
}
