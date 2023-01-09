// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.robot.util.GeometryUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kLoopTime = 0.02;
    public static final double HALF_PI = Math.PI / 2;
    public static final double TWO_PI = 2 * Math.PI;
    public static final boolean kTuningMode = true;
    public static final String kAprilTagFieldLayoutFilename = "2023-taglayout.json";
    public static final Trigger kEmptyTrigger = new Trigger(() -> false);

    public static final class BuildConstants {
        public static final double kRobotWeightKg = Units.lbsToKilograms(100);

        /**
         * Can be found from the CAD model
         */
        public static final double kRobotMomentOfInertia = 7.5;
    }

    public static final class ElectricalConstants {
        // Front Left Swerve Module Channels
        public static final int kFrontLeftTurnMotorPort = 23;
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontLeftCANCoderPort = 1;

        // Front Right Swerve Module Channels
        public static final int kFrontRightTurnMotorPort = 38;
        public static final int kFrontRightDriveMotorPort = 39;
        public static final int kFrontRightCANCoderPort = 3;

        // Back Left Swerve Module Channels
        public static final int kBackLeftTurnMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 16;
        public static final int kBackLeftCANCoderPort = 0;

        // Back Right Swerve Module Channels
        public static final int kBackRightTurnMotorPort = 32;
        public static final int kBackRightDriveMotorPort = 3;
        public static final int kBackRightCANCoderPort = 2;

        // Gyro Channel
        public static final int kGyroPort = 0;
    }

    public static final class DriveConstants {
        // Chassis Measurements
        public static final double kTrackWdith = Units.inchesToMeters(22.5);
        public static final double kWheelBase = Units.inchesToMeters(22.5);

        // Wheel Measurements
        public static final double kWheelDiameter = Units.inchesToMeters(3.7);

        // Gear Ratios
        public static final double kDriveGearRatio = 8.33;
        public static final double kTurnGearRatio = 18.0;

        // Encoder Conversion Factors
        public static final double kDrivePositionConversionFactor = Math.PI * kWheelDiameter / kDriveGearRatio;
        public static final double kTurnPositionConversionFactor = TWO_PI / kTurnGearRatio;

        // PID Values
        public static final double kDriveP = 5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.1;
        public static final double kDriveFF = 1.0;

        public static final double kTurnP = 0.01;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.005;

        // Max Linear Movement Values
        public static final double kMaxSpeedMetersPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSq = 3;

        // Max Angular Movement Values
        public static final double kMaxSpeedRadiansPerSecond = Units.degreesToRadians(360);
        public static final double kMaxAccelerationRadiansPerSecondSq = Units.degreesToRadians(20);

        // Max Linear Movement Values
        public static final double kAutoMaxSpeedMetersPerSecond = 8;
        public static final double kAutoMaxAccelerationMetersPerSecondSq = 3;
    }

    public static final class FieldConstants {
        public static final double kFieldWidthMeters = 16.46;
        public static final double kFieldHeightMeters = 8.23;

        public static final Pose3d kOppositeField = new Pose3d(kFieldWidthMeters, kFieldHeightMeters, 0,
                new Rotation3d(0, 0, Units.degreesToRadians(180)));

        public static Pose3d getFieldMirroredPose(Pose3d pose) {
            return kOppositeField.transformBy(GeometryUtils.poseToTransform(pose));
        }

        public static final double kHubXPos = kFieldWidthMeters / 2;
        public static final double kHubYPos = kFieldHeightMeters / 2;
        public static final Pose3d kHubPose = GeometryUtils.from2dTo3d(new Pose2d(
                kHubXPos, kHubYPos, Rotation2d.fromDegrees(24)));
    }

    public static class VisionConstants {
        /**
        * Name of the camera in photonvision
        * http://gloworm.local:5800
        */
        public static final String kCameraName = "camera";

        /**
         * Max camera yaw at the right (or -left) edge of the frame
         */
        public static final double kCameraYawFOV = 25.0;

        /**
         * Photonvision pipeline indices
         * http://gloworm.local:5800
         */
        public static final int kPipelineApriltagsExample = 0;
        public static final int kPipelineReflectiveTapeExample = 1;

        /**
         * Set containing all above photonvision pipelines which require LEDs to work correctly
         */
        public static final Set<Integer> kPipelinesRequiringLEDs = Collections.unmodifiableSet(Set.of(
                kPipelineReflectiveTapeExample // Example Target with Reflective Tape
        ));

        /**
         * Camera Position Relative to robot
         */
        public static final Translation3d kCameraOffset = new Translation3d(
                Units.inchesToMeters(10), // -front to +back
                Units.inchesToMeters(0), // -left to +right
                Units.inchesToMeters(-25));

        public static final double kCameraAngleYawOffset = Units.degreesToRadians(0.0); // left to right rotation
        public static final double kCameraAnglePitchOffset = Units.degreesToRadians(0.0); // up and down rotation

        public static final Transform3d kCameraToRobot = new Transform3d(
                kCameraOffset,
                GeometryUtils.yawPitchRotation(kCameraAngleYawOffset, kCameraAnglePitchOffset));
    }

    public static class AutoConstants {
        public static final String kAutoStatusKey = "Auto Status";

        public static final String kRightConeCone = "Right Cone Cone";
        public static final String kRightCubeCone = "Right Cone Cone";
        public static final String kFiveBallAuto = "";
        public static final String kDefaultAuto = kRightConeCone;

        public static final List<String> kAutoNames = PathPlannerUtil.getExistingPaths();

        // public static final List<String> kAutoNames = List.of(
        //         kRightConeCone,
        //         kRightCubeCone,
        //         kFiveBallAuto);
    }

    public static class LoggingConstants {
        public static final String kAprilTagPoses = "AprilTag Poses";
        public static final String kAprilTagIds = "AprilTag ID";
    }
}
