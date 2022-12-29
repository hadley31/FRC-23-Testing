# Generic Swerve Drive Commands
This java package contains swerve drive commands that are compatible with any swerve code base.

## Getting Started
There are a lot of drive classes here, so let me show you where to start:

```java
// Create your subsystem and joystick like normal
MyDriveSubsystem drive = new MyDriveSubsystem();

XboxController controller = new XboxController();

// Now create your swerve drive command config
DriveCommandConfig driveConfig = new DriveCommandConfig(
    drive::getPose,
    drive::setSpeeds,
    drive
);

// Create the drive command with your config
BaseDriveCommand driveCommand = BaseDriveCommand
    .create(driveConfig)
    .withSpeedSuppliers(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX())
    .withFieldOriented(true);

drive.setDefaultCommand(driveCommand);
```

## More Advanced Examples
That was a pretty simple example. It will provide you a start to driving around your swerve drive base. Now how do we do more complex things like facing a target while we drive?

```java
PIDController turnController = new PIDController(kP, kI, kD);
Pose2d hubPose = new Pose2d(8.23, 4.115, Rotation2d.from(24));

BaseDriveCommand driveCommand = new FaceTargetDrive(driveConfig, turnController, hubPose)
    .withLinearSpeedSuppliers(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());
```
