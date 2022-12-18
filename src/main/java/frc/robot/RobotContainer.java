// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.photonvision.SimPhotonCamera;

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
import frc.lib.vision.photonvision.SimVisionSystem;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.commands.drive.driver.DriveCommandBoundaryWrapper;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverXboxControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.chassis.CompetitionChassis;
import frc.robot.subsystems.drive.chassis.DriveChassis;
import frc.robot.subsystems.drive.gyro.PigeonIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIO;
import frc.robot.subsystems.drive.modules.SwerveModuleIOSim;
import frc.robot.subsystems.vision.Camera;
import frc.robot.ui.GlassInterface;
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
    private AutoFactory m_autoFactory = new AutoFactory(m_drive, m_camera);

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
        } catch (Exception e) {
            System.out.println("Unable to load apriltag field layout");
        }
    }

    private void configureSubsystems() {
        if (RobotBase.isSimulation()) {
            m_camera = new Camera(new SimPhotonCamera(VisionConstants.kCameraName));
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
            //     m_camera = new Camera(new PhotonCamera(VisionConstants.kCameraName));
            //     DriveChassis chassis = new CompetitionChassis(
            //             new SwerveModuleIO[] {
            //                     new SwerveModuleIOMK2Neo(
            //                             ElectricalConstants.kFrontLeftTurnMotorPort,
            //                             ElectricalConstants.kFrontLeftDriveMotorPort,
            //                             ElectricalConstants.kFrontLeftCANCoderPort),
            //                     new SwerveModuleIOMK2Neo(
            //                             ElectricalConstants.kFrontRightTurnMotorPort,
            //                             ElectricalConstants.kFrontRightDriveMotorPort,
            //                             ElectricalConstants.kFrontRightCANCoderPort),
            //                     new SwerveModuleIOMK2Neo(
            //                             ElectricalConstants.kBackLeftTurnMotorPort,
            //                             ElectricalConstants.kBackLeftDriveMotorPort,
            //                             ElectricalConstants.kBackLeftCANCoderPort),
            //                     new SwerveModuleIOMK2Neo(
            //                             ElectricalConstants.kBackRightTurnMotorPort,
            //                             ElectricalConstants.kBackRightDriveMotorPort,
            //                             ElectricalConstants.kBackRightCANCoderPort),
            //             },
            //             new WPI_Pigeon2(ElectricalConstants.kGyroPort));
            //     m_drive = new Drive(
            //             chassis,
            //             new PoseEstimator(chassis, m_camera));
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

        var boundedJoystickDriveCommand = new DriveCommandBoundaryWrapper(m_drive.driveCommand(driverControls));

        // Define subsystem default commands
        m_drive.setDefaultCommand(boundedJoystickDriveCommand);

        // Define Driver Control Mappings
        driverControls.getRobotRelativeDriveMode().whileTrue(boundedJoystickDriveCommand.withFieldRelative(false));

        // Define Operator Control Mappings
        // operatorControls.getExampleControl().whenActive(new PrintCommand("Operator did a thing!"));
    }

    /**
     * Loads the auto path from the json path file when the robot is first initialized
     */
    public void configureAuto() {
        m_autoFactory = new AutoFactory(m_drive, m_camera);

        m_autoFactory.loadAutoPathByName("5 Ball Auto");
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // int targetFiducial = 6;
        // var targetOffset = new Transform2d(new Translation2d(2, 0), Rotation2d.fromDegrees(180));
        // return new FollowTarget(m_drive, m_camera, 0, targetFiducial, targetOffset);

        return m_autoFactory.getAutoCommand();
    }

    public void onDisable() {
        DebugCommands.brakeAndReset(m_drive).schedule();
    }

    /**
     * Simulation
     */

    private SimVisionSystem m_simVision;
    private NotSoPeriodic m_cameraNotSoPeriodic;

    public void simulationInit() {
        m_cameraNotSoPeriodic = new NotSoPeriodic(1);
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

        GlassInterface.setObjectPoses("AprilTags",
                tags.values().stream().map(x -> x.toPose2d()).toArray(Pose2d[]::new));
        // GlassInterface.setObjects3d(tags);
    }

    public void simulationPeriodic() {
        m_cameraNotSoPeriodic.call(() -> {
            m_simVision.processFrame(m_drive.getPose());
        });

        GlassInterface.setSwerveRobotPose(m_drive.getPose(), m_drive.getChassis());
    }

    /**
     * End Simulation
     */
}
