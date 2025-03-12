package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
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

    public static Command snapToBranch(Camera camera, int id, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double maxSpeed)
    {
        return Commands.either(autoAlign(camera, id, id % 2 == 0 ? new Pose2d() : new Pose2d()), DriveCommands.driveAtOrientation(xSupplier, ySupplier, robotCentric, () -> Constants.Field.getTagAngle(id), Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE), null); // Pose2d's need updated
    }

    public static Command autoAlign(Camera camera, int id, Pose2d reference)
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.runOnce(() -> Vision.getInstance(camera).setVisionReference(id, reference)),
            joystickDrive(() -> Vision.getInstance(camera).getXDistanceCalculation(), () -> Vision.getInstance(camera).getYDistanceCalculation(), () -> Vision.getInstance(camera).getAngleCalculation(), () -> true, 1, 1)
        );
        // @formatter:on
    }

    public static Command intake()
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.repeatingSequence
            (
                ManipulatorCommands.intake()
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
            ManipulatorCommands.output(),
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
