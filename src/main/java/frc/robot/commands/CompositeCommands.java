package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Field.Branch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Camera;

public class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command joystickDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotCentric, int translateExponent, double rotateExponent)
    {
        return Commands.run(() ->
        {
            // Apply deadband
            double     linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), Constants.Controls.JOYSTICK_DEADBAND);
            Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            double     omega           = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Controls.JOYSTICK_DEADBAND);
            double     speedModifier   = MathUtil
                    .clamp((Constants.Drive.SPEED_ELEVATOR_M * Elevator.getInstance().getExtension() + Constants.Drive.SPEED_ELEVATOR_B), Constants.Drive.MIN_SPEED_ELEVATOR_MULTIPLIER, Constants.Drive.MAX_SPEED_ELEVATOR_MULTIPLIER);

            // Square values
            linearMagnitude = Math.pow(linearMagnitude, translateExponent);
            omega           = Math.copySign(Math.pow(Math.abs(omega), rotateExponent), omega);

            // Calculate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Convert to field relative speeds & send command
            if (robotCentric.getAsBoolean())
            {
                var chassisSpeeds = new ChassisSpeeds(
                        linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, omega * Constants.Drive.MAX_ANGULAR_SPEED * speedModifier
                );

                Drive.getInstance().runVelocity(chassisSpeeds);
            }
            else
            {
                var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, omega * Constants.Drive.MAX_ANGULAR_SPEED * speedModifier,
                        Drive.getInstance().getRotation()
                );

                Drive.getInstance().runVelocity(chassisSpeeds);
            }
        }, Drive.getInstance());
    }

    public static Command autoAlign(Branch branch, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double translateMaxSpeed, double rotateMaxSpeed)
    {
        // @formatter:off
        return Commands.defer(() ->
            Commands.sequence
            (
                Commands.runOnce(() -> {
                    Drive.getInstance().rotateInit(Constants.Field.getTagAngle(branch.getID()), rotateMaxSpeed);
                    Vision.getInstance(Camera.FrontCenter).setVisionReference(branch, translateMaxSpeed);
                }),

                DriveCommands.driveToPose(Drive.getInstance().getPose().plus(Vision.getInstance(Camera.FrontCenter).getHorizontalTranslation()), translateMaxSpeed, rotateMaxSpeed)
            ),
            Set.of(Drive.getInstance())
        );
        // @formatter:off
    }

    // public static Command autoAlign(Branch branch, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double translateMaxSpeed, double rotateMaxSpeed)
    // {
    //     // @formatter:off
    //     return Commands.defer(() ->
    //         Commands.sequence
    //         (
    //             Commands.runOnce(() -> 
    //             {
    //                 Drive.getInstance().rotateInit(Constants.Field.getTagAngle(branch.getID()), rotateMaxSpeed);
    //                 Vision.getInstance(Camera.FrontCenter).setVisionReference(branch, translateMaxSpeed);
    //             }),

    //             Commands.run(() ->
    //             {
    //                 double x;
    //                 double y;
    //                 boolean isRobotCentric;
    //                 double translateExponent;

    //                 if (Vision.getInstance(Camera.FrontCenter).hasTarget())
    //                 {
    //                     x = 0;
    //                     y = Vision.getInstance(Camera.FrontCenter).alignExecute();
    //                     isRobotCentric = true;
    //                     translateExponent = 1;
    //                 }
    //                 else
    //                 {
    //                     x = xSupplier.getAsDouble();
    //                     y = ySupplier.getAsDouble();
    //                     isRobotCentric = robotCentric.getAsBoolean();
    //                     translateExponent = 2;
    //                 }

    //                 // Apply deadband
    //                 double     linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0);
    //                 Rotation2d linearDirection = new Rotation2d(x, y);
    //                 double     omega           = MathUtil.applyDeadband(Drive.getInstance().rotateExecute(), Constants.Controls.JOYSTICK_DEADBAND);
    //                 double     speedModifier   = MathUtil
    //                         .clamp((Constants.Drive.SPEED_ELEVATOR_M * Elevator.getInstance().getExtension() + Constants.Drive.SPEED_ELEVATOR_B), Constants.Drive.MIN_SPEED_ELEVATOR_MULTIPLIER, Constants.Drive.MAX_SPEED_ELEVATOR_MULTIPLIER);

    //                 // Square values
    //                 linearMagnitude = Math.pow(linearMagnitude, translateExponent);
    //                 omega           = Math.copySign(Math.pow(Math.abs(omega), 1), omega);

    //                 // Calculate new linear velocity
    //                 Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

    //                 // Convert to field relative speeds & send command
    //                 if (isRobotCentric)
    //                 {
    //                     var chassisSpeeds = new ChassisSpeeds(
    //                             linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, omega * Constants.Drive.MAX_ANGULAR_SPEED * speedModifier
    //                     );

    //                     Drive.getInstance().runVelocity(chassisSpeeds);
    //                 }
    //                 else
    //                 {
    //                     var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //                             linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED * speedModifier, omega * Constants.Drive.MAX_ANGULAR_SPEED * speedModifier,
    //                             Drive.getInstance().getRotation()
    //                     );

    //                     Drive.getInstance().runVelocity(chassisSpeeds);
    //                 }
    //             })
    //             .until(() -> Vision.getInstance(Camera.FrontCenter).alignIsFinished()),

    //             joystickDrive(() -> translateMaxSpeed, () -> 0, () -> Drive.getInstance().rotateExecute(), () -> true, 1, 1)
    //             .until(() -> Drive.getInstance().collisionDetected()),

    //             DriveCommands.stop(),

    //             Commands.idle(Drive.getInstance())
    //         ),
    //         Set.of(Drive.getInstance(), Vision.getInstance(Camera.FrontCenter))
    //     );
    //     // @formatter:on
    // }

    // public static Command autoAlign(Branch branch, double translateMaxSpeed, double rotateMaxSpeed)
    // {
    //     // @formatter:off
    //     return Commands.defer(() ->
    //         Commands.sequence
    //         (
    //             Commands.runOnce(() -> 
    //             {
    //                 Drive.getInstance().rotateInit(Constants.Field.getTagAngle(branch.getID()), rotateMaxSpeed);
    //                 Vision.getInstance(Camera.FrontCenter).setVisionReference(branch, translateMaxSpeed);
    //             }),

    //             joystickDrive(() -> 0, () -> Vision.getInstance(Camera.FrontCenter).alignExecute(), () -> Drive.getInstance().rotateExecute(), () -> true, 1, 1)
    //             .until(() -> Vision.getInstance(Camera.FrontCenter).alignIsFinished() || !Vision.getInstance(Camera.FrontCenter).hasTarget()),

    //             joystickDrive(() -> translateMaxSpeed, () -> 0, () -> Drive.getInstance().rotateExecute(), () -> true, 1, 1)
    //             .until(() -> Drive.getInstance().collisionDetected()),

    //             DriveCommands.stop()
    //         ),

    //         Set.of(Drive.getInstance(), Vision.getInstance(Camera.FrontCenter))
    //     );
    //     // @formatter:on
    // }

    // public static Command snapToBranch(Camera camera, Branch branch,
    // DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier
    // robotCentric, double translationMaxSpeed, double rotationMaxSpeed)
    // {
    //     // @formatter:off
    //     return Commands.defer
    //     (
    //         () -> Commands.sequence
    //         (
    //             Commands.runOnce(() -> 
    //             {
    //                 Vision.getInstance(camera).setVisionReference(branch.getID());
    //                 Logger.recordOutput("AutoAlign/Target", Utilities.getTagPose(branch.getID()).rotateAround(Utilities.getTagPose(branch.getID()).getTranslation(), Rotation2d.fromDegrees(180)).transformBy(new Transform2d(branch.getReference(), new Rotation2d())));
    //             }), 
    //             DriveCommands.driveAtOrientation(xSupplier, ySupplier, robotCentric, Constants.Field.getTagAngle(branch.getID()), rotationMaxSpeed).until(() -> Vision.getInstance(camera).hasTarget(branch.getID())),
    //             DriveCommands.driveToPose(Utilities.getTagPose(branch.getID()).rotateAround(Utilities.getTagPose(branch.getID()).getTranslation(), Rotation2d.fromDegrees(180)).transformBy(new Transform2d(branch.getReference(), new Rotation2d())), translationMaxSpeed, rotationMaxSpeed)
    //         )
    //         .finallyDo(() -> Vision.getInstance(Camera.Front).setVisionReference(0)),
    //         Set.of(Drive.getInstance())
    //     );
    //     // @formatter:on
    // }

    // public static Command snapToBranchAuto(Camera camera, Branch branch, double
    // translationMaxSpeed, double rotationMaxSpeed)
    // {
    //     // @formatter:off
    //     return Commands.defer
    //     (
    //         () -> Commands.sequence
    //         (
    //             Commands.runOnce(() -> Logger.recordOutput("AutoAlign/Target", Utilities.getTagPose(branch.getID()).rotateAround(Utilities.getTagPose(branch.getID()).getTranslation(), Rotation2d.fromDegrees(180)).transformBy(new Transform2d(branch.getReference(), new Rotation2d())))),
    //             DriveCommands.driveToPose(Utilities.getTagPose(branch.getID()).rotateAround(Utilities.getTagPose(branch.getID()).getTranslation(), Rotation2d.fromDegrees(180)).transformBy(new Transform2d(branch.getReference(), new Rotation2d())), translationMaxSpeed, rotationMaxSpeed)
    //             .until(() -> Drive.getInstance().isAligned()))
    //             .finallyDo(() -> Vision.getInstance(Camera.Front).setVisionReference(0)),
    //             Set.of(Drive.getInstance()
    //         )
    //     );
    //     // @formatter:on
    // }

    public static Command intake()
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.repeatingSequence
            (
                ManipulatorCommands.coralIntake()
            )
            .until(() -> Manipulator.getInstance().hasCoral()),

            ManipulatorCommands.index()
        )
        .unless(() -> Manipulator.getInstance().hasCoral());
        // @formatter:on
    }

    public static Command output()
    {
        // @formatter:off
        return Commands.sequence
        (
            ManipulatorCommands.coralOutput(),
            Commands.waitSeconds(Constants.Elevator.WAIT_TIME),
            ElevatorCommands.setHeight(ElevatorHeight.Stow)
        );
        // @formatter:on
    }

    public static Command setHeight(ElevatorHeight height)
    {
        // @formatter:off
        return Commands.sequence
        (
            ElevatorCommands.setHeight(height),
            Commands.waitUntil(() -> Elevator.getInstance().atSetpoint())
        );
        // @formatter:on
    }
}
