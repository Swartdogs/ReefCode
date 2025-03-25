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
import frc.robot.util.Utilities;

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
                Commands.runOnce(() -> Vision.getInstance(Camera.FrontCenter).setVisionReference(branch)),

                DriveCommands.driveAtOrientation(xSupplier, ySupplier, robotCentric, Constants.Field.getTagAngle(branch.getID()), rotateMaxSpeed)
                .until(() -> Vision.getInstance(Camera.FrontCenter).hasTarget()),

                DriveCommands.driveToPose(Utilities.getTagPose(branch.getID()).rotateAround(Utilities.getTagPose(branch.getID()).getTranslation(), Rotation2d.fromDegrees(180)).transformBy(new Transform2d(branch.getReference(), new Rotation2d())), translateMaxSpeed, rotateMaxSpeed)
                .until(() -> (Drive.getInstance().xDriveIsFinished() && Drive.getInstance().yDriveIsFinished() && Drive.getInstance().rotateIsFinished())),

                DriveCommands.joystickDrive(() -> 0, () -> Drive.getInstance().yDriveExecute(), () -> Drive.getInstance().rotateExecute(), () -> true, 1, 1)
                .until(() -> Drive.getInstance().yDriveIsFinished() && Drive.getInstance().rotateIsFinished()),

                DriveCommands.stop(),

                Commands.idle(Drive.getInstance())
            )
            .finallyDo(() -> Drive.getInstance().stop()),
            Set.of(Drive.getInstance())
        );
        // @formatter:off
    }

    public static Command autoAlign(Branch branch, double translateMaxSpeed, double rotateMaxSpeed)
    {
        // @formatter:off
        return Commands.defer(() ->
            Commands.sequence
            (
                Commands.runOnce(() -> 
                {
                    Vision.getInstance(Camera.FrontCenter).setVisionReference(branch);
                    Drive.getInstance().rotateInit(Constants.Field.getTagAngle(branch.getID()), rotateMaxSpeed);
                }),

                DriveCommands.driveToPose(Utilities.getTagPose(branch.getID()).rotateAround(Utilities.getTagPose(branch.getID()).getTranslation(), Rotation2d.fromDegrees(180)).transformBy(new Transform2d(branch.getReference(), new Rotation2d())), translateMaxSpeed, rotateMaxSpeed)
                .until(() -> (Drive.getInstance().xDriveIsFinished() && Drive.getInstance().yDriveIsFinished() && Drive.getInstance().rotateIsFinished()))
            )
            .finallyDo(() -> Drive.getInstance().stop()),

            Set.of(Drive.getInstance())
        );
        // @formatter:off
    }

    
    public static Command coralIntake()
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.repeatingSequence
            (
                ElevatorCommands.setHeight(ElevatorHeight.Stow).unless(() -> Elevator.getInstance().getSetpoint() == null),
                ManipulatorCommands.coralIntake()
                // ElevatorCommands.setHeight(ElevatorHeight.Coast)
            )
            .until(() -> Manipulator.getInstance().hasCoral()),

            ManipulatorCommands.index()
        )
        .unless(() -> Manipulator.getInstance().hasCoral());
        // @formatter:on
    }

    public static Command coralOutput()
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
        return 
        Commands.either
        (
            Commands.sequence
            (
                ElevatorCommands.setHeight(height), 
                Commands.waitUntil(() -> Elevator.getInstance().atSetpoint())
            ),
            Commands.none(), 
            () -> !height.getDisableable() || Manipulator.getInstance().coralSafe()
        );
        // @formatter:on
    }
}
