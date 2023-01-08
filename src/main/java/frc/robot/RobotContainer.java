// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.SimVisionSystem;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.listeners.ChangeNotifier;
import frc.lib.vision.photonvision.PhotonCamera;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.commands.debug.DriveBoundary;
import frc.robot.commands.debug.DriveBoundary.Bounds;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverXboxControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.chassis.CompetitionChassis;
import frc.robot.subsystems.drive.chassis.DriveChassis;
import frc.robot.subsystems.drive.gyro.PigeonIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIOMK2Neo;
import frc.robot.subsystems.drive.modules.SwerveModuleIOSim;
import frc.robot.subsystems.vision.Camera;
import frc.robot.util.FieldUtil;
import frc.robot.util.NotSoPeriodic;
import frc.robot.util.PoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private Drive m_drive;
    private Camera m_camera;

    private AprilTagFieldLayout m_tagLayout;
    private AutoFactory m_autoFactory;
    private LoggedDashboardChooser<String> m_autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureAprilTags(DriverStation.getAlliance());
        configureSubsystems();
        configureButtonBindings();
        configureAuto();
    }

    private void configureAprilTags(Alliance alliance) {
        try {
            System.out.printf("Setting apriltag positions using %s alliance.\n", alliance);
            var originPosition = alliance == Alliance.Red
                    ? OriginPosition.kRedAllianceWallRightSide
                    : OriginPosition.kBlueAllianceWallRightSide;

            m_tagLayout = new AprilTagFieldLayout(Constants.kAprilTagFieldLayoutFilename);
            m_tagLayout.setOrigin(originPosition);

            Logger.getInstance().recordOutput(
                    "AprilTag ID",
                    m_tagLayout.getTags().stream().mapToLong(x -> x.ID).toArray());

            Logger.getInstance().recordOutput(
                    "AprilTag Poses",
                    m_tagLayout.getTags().stream().map(x -> m_tagLayout.getTagPose(x.ID).get()).toArray(Pose3d[]::new));
        } catch (Exception e) {
            System.out.println("Unable to load apriltag field layout");
        }
    }

    private void configureSubsystems() {
        if (RobotBase.isSimulation()) {
            m_camera = new Camera(new PhotonCamera(VisionConstants.kCameraName));
            DriveChassis chassis = new CompetitionChassis(
                    new SwerveModuleIO[] {
                            new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(),
                    },
                    new PigeonIO(ElectricalConstants.kGyroPort));
            m_drive = new Drive(
                    chassis,
                    new PoseEstimator(chassis, m_camera, m_tagLayout));
        } else {
            m_camera = new Camera(new PhotonCamera(VisionConstants.kCameraName));
            DriveChassis chassis = new CompetitionChassis(
                    new SwerveModuleIO[] {
                            new SwerveModuleIOMK2Neo(
                                    ElectricalConstants.kFrontLeftTurnMotorPort,
                                    ElectricalConstants.kFrontLeftDriveMotorPort,
                                    ElectricalConstants.kFrontLeftCANCoderPort),
                            new SwerveModuleIOMK2Neo(
                                    ElectricalConstants.kFrontRightTurnMotorPort,
                                    ElectricalConstants.kFrontRightDriveMotorPort,
                                    ElectricalConstants.kFrontRightCANCoderPort),
                            new SwerveModuleIOMK2Neo(
                                    ElectricalConstants.kBackLeftTurnMotorPort,
                                    ElectricalConstants.kBackLeftDriveMotorPort,
                                    ElectricalConstants.kBackLeftCANCoderPort),
                            new SwerveModuleIOMK2Neo(
                                    ElectricalConstants.kBackRightTurnMotorPort,
                                    ElectricalConstants.kBackRightDriveMotorPort,
                                    ElectricalConstants.kBackRightCANCoderPort),
                    },
                    new PigeonIO(ElectricalConstants.kGyroPort));
            m_drive = new Drive(
                    chassis,
                    new PoseEstimator(chassis, m_camera, m_tagLayout));
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        DriverControls driverControls = new DriverXboxControls(0);
        // DriverControls driverControls = new DriverJoystickControls(0, 1);

        // OperatorControls operatorControls = new OperatorXboxControls(2);

        var driveBounds = new Bounds(2, 2, 6, 4);
        var boundedJoystickDriveCommand = new DriveBoundary(
                m_drive.driveCommand(driverControls),
                driveBounds);

        // Define subsystem default commands
        m_drive.setDefaultCommand(boundedJoystickDriveCommand);

        // Define Driver Control Mappings
        driverControls.getRobotRelativeDriveMode()
                .whileTrue(boundedJoystickDriveCommand.withFieldRelative(false));

        // Define Operator Control Mappings
        // operatorControls.getExampleControl().whenActive(new PrintCommand("Operator did a thing!"));
    }

    /**
     * Loads the auto path from the json path file when the robot is first initialized.
     * This is so that the robot does not need to do any slow loading of files just as
     * autonomous starts
     */
    public void configureAuto() {
        m_autoFactory = new AutoFactory(m_drive, m_camera);
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

        // Configure Auto Chooser Options
        AutoConstants.kAutoNames.forEach(name -> m_autoChooser.addOption(name, name));
        m_autoChooser.addDefaultOption(AutoConstants.kDefaultAuto, AutoConstants.kDefaultAuto);

        ChangeNotifier.of(m_autoChooser::get)
                .addListener(x -> {
                    System.out.println("Loading auto: " + x);
                    m_autoFactory.loadAutoPathByName(x);
                });

        // Load the default auto
        m_autoFactory.loadAutoPathByName(AutoConstants.kDefaultAuto);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (Robot.isSimulation()) {
            // Reload from file if we're in simulation to get latest changes
            m_autoFactory.loadSelectedPathFromFile();
        }

        return m_autoFactory.getAutoCommand();
    }

    public void onDisable() {
        DebugCommands.brakeAndReset(m_drive).schedule();
    }

    //#region Simulation

    private SimVisionSystem m_simVision;
    private NotSoPeriodic m_cameraNotSoPeriodic;

    public void simulationInit() {
        // Disable joystick warning in simulator
        DriverStation.silenceJoystickConnectionWarning(true);

        m_cameraNotSoPeriodic = new NotSoPeriodic(1);

        // Create simulated vision system
        m_simVision = new SimVisionSystem(
                VisionConstants.kCameraName,
                70,
                VisionConstants.kCameraToRobot,
                Units.feetToMeters(15),
                640, 480,
                1.0);
        m_simVision.addVisionTargets(m_tagLayout);

        HashMap<String, Pose3d> tags = new HashMap<>();
        for (var tag : m_tagLayout.getTags()) {
            tags.put("AprilTag_" + tag.ID, tag.pose);
        }

        FieldUtil.getDefaultField().setObjectPoses("AprilTags",
                tags.values().stream().map(x -> x.toPose2d()).toArray(Pose2d[]::new));
    }

    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
        m_cameraNotSoPeriodic.call(() -> {
            m_simVision.processFrame(m_drive.getPose());
        });
    }

    //#endregion
}
