// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DualJoystickDriverControls;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.XboxOperatorControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.PigeonIO;
import frc.robot.subsystems.drive.module.SwerveModuleIOMK4iNeo;
import frc.robot.subsystems.drive.module.SwerveModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIONeo;
import frc.robot.subsystems.elevator.ElevatorIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive m_drive;
  private Elevator m_elevator;

  private Mechanism2d m_mechanism;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSubsystems();
    configureButtonBindings();
  }

  private void configureSubsystems() {
    m_mechanism = new Mechanism2d(70, 70);

    var root = m_mechanism.getRoot("robot", 5, 5);
    var elevatorLigament = root.append(new MechanismLigament2d("elevator", 0, Elevator.kElevatorAngle.getDegrees()));
    var staticArmLigament = elevatorLigament
        .append(new MechanismLigament2d("staticArm", 16, -Elevator.kElevatorAngle.getDegrees(), 10,
            new Color8Bit(Color.kBlue)));
    var wristLigament = staticArmLigament
        .append(new MechanismLigament2d("wrist", 4, 0, 10, new Color8Bit(Color.kAliceBlue)));

    if (Robot.isReal()) {
      m_drive = new Drive(
          new PigeonIO(ElectricalConstants.kGyroPort),
          new SwerveModuleIOMK4iNeo(
              ElectricalConstants.kFrontLeftTurnMotorPort,
              ElectricalConstants.kFrontLeftDriveMotorPort,
              ElectricalConstants.kFrontLeftCANCoderPort),
          new SwerveModuleIOMK4iNeo(
              ElectricalConstants.kFrontRightTurnMotorPort,
              ElectricalConstants.kFrontRightDriveMotorPort,
              ElectricalConstants.kFrontRightCANCoderPort),
          new SwerveModuleIOMK4iNeo(
              ElectricalConstants.kBackLeftTurnMotorPort,
              ElectricalConstants.kBackLeftDriveMotorPort,
              ElectricalConstants.kBackLeftCANCoderPort),
          new SwerveModuleIOMK4iNeo(
              ElectricalConstants.kBackRightTurnMotorPort,
              ElectricalConstants.kBackRightDriveMotorPort,
              ElectricalConstants.kBackRightCANCoderPort));
      m_elevator = new Elevator(new ElevatorIONeo(
          ElectricalConstants.kElevatorLeaderPort,
          ElectricalConstants.kElevatorFollowerPort), elevatorLigament);
    } else {
      m_drive = new Drive(
          new PigeonIO(ElectricalConstants.kGyroPort),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());
      m_elevator = new Elevator(new ElevatorIOSim(), elevatorLigament);
    }

    SmartDashboard.putData("Robot Mechanism", m_mechanism);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // var singleUserControls = new SingleUserXboxControls(5);
    // DriverControls driverControls = singleUserControls;
    // OperatorControls operatorControls = singleUserControls;

    DriverControls driverControls = new DualJoystickDriverControls(0, 1);
    OperatorControls operatorControls = new XboxOperatorControls(5);

    DriveCommandConfig driveConfig = new DriveCommandConfig(
        DriveConstants.kDriveKinematics,
        m_drive::getPose,
        m_drive::getModuleStates,
        m_drive.getGyro()::getRate,
        m_drive::drive,
        m_drive);

    BaseDriveCommand fieldRelativeDrive = DriveCommands.fieldRelativeDrive(driveConfig, driverControls);
    BaseDriveCommand robotRelativeDrive = DriveCommands.robotRelativeDrive(driveConfig, driverControls);
    BaseDriveCommand joystickAngleDrive = DriveCommands.joystickAngleDrive(driveConfig, driverControls);

    m_drive.setDefaultCommand(fieldRelativeDrive);
    driverControls.robotRelativeDrive().whileTrue(robotRelativeDrive);
    driverControls.joystickAngleDrive().whileTrue(joystickAngleDrive);

    driverControls.testButton().onTrue(ChargeStationBalance.charge(m_drive));
    driverControls.resetPose().onTrue(m_drive.resetPoseCommand(new Pose2d(5, 3, Rotation2d.fromDegrees(0))));

    operatorControls.getExampleOperatorButton().onTrue(Commands.print("Operator pressed a button!"));
    operatorControls.zeroTurnAbsoluteEncoders().onTrue(DebugCommands.zeroTurnAbsoluteEncoders(m_drive));

    // Elevator Buttons
    operatorControls.setElevatorPositionHigh().onTrue(m_elevator.fullExtendCommand());
    operatorControls.setElevatorPositionMid().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(3)));
    operatorControls.setElevatorPositionLow().onTrue(m_elevator.fullRetractCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public void periodic() {
    Logger.getInstance().recordOutput("Mechanism2d", m_mechanism);
  }

  public void onDisabled() {
    DebugCommands.brakeAndReset(m_drive).schedule();
  }
}
