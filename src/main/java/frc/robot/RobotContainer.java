// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.SimVisionSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.SingleUserXboxControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.module.SwerveModuleIOSim;
import frc.robot.subsystems.drive.module.SwerveModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.AprilTagCamera;

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
  private AprilTagCamera m_camera;

  private AprilTagFieldLayout m_tagLayout;

  private AutoFactory m_autoFactory;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureAprilTags();
    configureAllianceSettings(DriverStation.getAlliance());
    configureSubsystems();
    configureButtonBindings();
    configureAuto();
  }

  private void configureAuto() {
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoFactory = new AutoFactory(m_drive, m_camera, m_elevator);

    // Add basic autonomous commands
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addOption("Zero Absolute Encoders", DebugCommands.zeroTurnAbsoluteEncoders(m_drive));

    // Add PathPlanner Auto Commands
    PathPlannerUtil.getExistingPaths().forEach(path -> {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    });
  }

  private void configureAprilTags() {
    try {
      m_tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

      if (Robot.isSimulation()) {
        m_simVisionSystem = Optional.of(
            new SimVisionSystem(
                VisionConstants.kAprilTagCameraGrayConfig.getName(),
                VisionConstants.kAprilTagCameraGrayConfig.getFOV(),
                VisionConstants.kAprilTagCameraGrayConfig.getRobotToCamera(),
                5,
                640, 480,
                10.0));
      }
    } catch (IOException e) {
      throw new RuntimeException("Unable to load apriltag field layout!");
    }
  }

  public void configureAllianceSettings(Alliance alliance) {
    var origin = alliance == Alliance.Red
        ? OriginPosition.kRedAllianceWallRightSide
        : OriginPosition.kBlueAllianceWallRightSide;
    m_tagLayout.setOrigin(origin);

    FieldUtil.getDefaultField().setObjectGlobalPoses("AprilTags",
        m_tagLayout.getTags().stream().map(x -> x.pose.toPose2d()).toArray(Pose2d[]::new));

    m_simVisionSystem.ifPresent(visionSystem -> {
      visionSystem.clearVisionTargets();
      visionSystem.addVisionTargets(m_tagLayout);
    });
  }

  private void configureSubsystems() {
    if (Robot.isReal()) {
      m_drive = new Drive(
          new GyroIONavX(Port.kMXP),
          new SwerveModuleIOTalonFX(
              ElectricalConstants.kFrontLeftTurnMotorPort,
              ElectricalConstants.kFrontLeftDriveMotorPort,
              ElectricalConstants.kFrontLeftCANCoderPort,
              Rotation2d.fromRadians(0.618)),
          new SwerveModuleIOTalonFX(
              ElectricalConstants.kFrontRightTurnMotorPort,
              ElectricalConstants.kFrontRightDriveMotorPort,
              ElectricalConstants.kFrontRightCANCoderPort,
              Rotation2d.fromRadians(2.966)),
          new SwerveModuleIOTalonFX(
              ElectricalConstants.kBackLeftTurnMotorPort,
              ElectricalConstants.kBackLeftDriveMotorPort,
              ElectricalConstants.kBackLeftCANCoderPort,
              Rotation2d.fromRadians(0.548)),
          new SwerveModuleIOTalonFX(
              ElectricalConstants.kBackRightTurnMotorPort,
              ElectricalConstants.kBackRightDriveMotorPort,
              ElectricalConstants.kBackRightCANCoderPort,
              Rotation2d.fromRadians(0.993)));
    } else {
      m_drive = new Drive(
          new GyroIONavX(Port.kMXP),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());
    }

    SmartDashboard.putData("Robot Mechanism", m_mechanism);
    m_camera = new AprilTagCamera(
        VisionConstants.kAprilTagCameraGrayConfig.getName(),
        VisionConstants.kAprilTagCameraGrayConfig.getRobotToCamera(),
        m_tagLayout,
        m_drive.getPoseEstimator());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    var singleUserControls = new SingleUserXboxControls(5);
    DriverControls driverControls = singleUserControls;
    OperatorControls operatorControls = singleUserControls;

    // DriverControls driverControls = new DualJoystickDriverControls(0, 1);
    // OperatorControls operatorControls = new XboxOperatorControls(5);

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
    driverControls.resetPoseToVisionEst().onTrue(m_drive.resetPoseCommand(m_camera::getLatestEstimatedPose));

    operatorControls.getExampleOperatorButton().onTrue(Commands.print("Operator pressed a button!"));
    operatorControls.zeroTurnAbsoluteEncoders().onTrue(DebugCommands.zeroTurnAbsoluteEncoders(m_drive));

    // Elevator Buttons
    operatorControls.setElevatorPositionHigh().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(5)));
    operatorControls.setElevatorPositionMid().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(3)));
    operatorControls.setElevatorPositionLow().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(1.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Robot.isSimulation()) {
      String selectedAuto = m_autoChooser.getSendableChooser().getSelected();
      if (PathPlannerUtil.getExistingPaths().contains(selectedAuto)) {
        System.out.println("Reloading pathplanner path file: " + selectedAuto);
        return m_autoFactory.getAutoCommand(selectedAuto);
      }
    }
    return m_autoChooser.get();
  }

  //#region Sim Stuff

  private Optional<SimVisionSystem> m_simVisionSystem = Optional.empty();

  public void simulationPeriodic() {
    m_simVisionSystem.ifPresent(visionSystem -> {
      visionSystem.processFrame(m_drive.getPose());
    });
  }

  //#endregion

  //#region Robot State Callbacks

  public void periodic() {
    Logger.getInstance().recordOutput("Mechanism2d", m_mechanism);
  }

  public void onDisabled() {
    if (Constants.kDebugMode) {
      DebugCommands.brakeAndReset(m_drive).schedule();
    }
  }

  //#endregion
}
