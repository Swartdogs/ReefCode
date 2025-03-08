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

public final class DriveCommands
{
    private DriveCommands()
    {
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotCentric)
    {
        return joystickDrive(xSupplier, ySupplier, omegaSupplier, robotCentric, 2, 3);
    }

    public static Command joystickDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotCentric, int translateExponent, double rotateExponent)
    {
        return Commands.run(() ->
        {
            // Apply deadband
            double     linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), Constants.Controls.JOYSTICK_DEADBAND);
            Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            double     omega           = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Controls.JOYSTICK_DEADBAND);

            // Square values
            linearMagnitude = Math.pow(linearMagnitude, translateExponent);
            omega           = Math.copySign(Math.pow(Math.abs(omega), rotateExponent), omega);

            // Calculate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Convert to field relative speeds & send command
            if (robotCentric.getAsBoolean())
            {
                var chassisSpeeds = new ChassisSpeeds(linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED);

                Drive.getInstance().runVelocity(chassisSpeeds);
            }
            else
            {
                Drive.getInstance().runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED, Drive.getInstance().getRotation()
                        )
                );
            }
        }, Drive.getInstance());
    }

    public static Command driveAtOrientation(DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, Supplier<Rotation2d> setpoint, double maxSpeed)
    {
        return Commands.runOnce(() -> Drive.getInstance().rotateInit(setpoint.get(), maxSpeed)).andThen(joystickDrive(xSupplier, ySupplier, () -> Drive.getInstance().rotateExecute(), robotCentric, 2, 1));
    }

    public static Command resetGyro()
    {
        return Commands.runOnce(() ->
        {
            var pose = Drive.getInstance().getPose();
            Drive.getInstance().setPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0)));
        }).ignoringDisable(true);
    }

    public static Command driveVolts(double volts)
    {
        return Commands.runOnce(() -> Drive.getInstance().runVolts(volts));
    }

    public static Command reduceSpeed()
    {
        return Commands.startEnd(() -> Drive.getInstance().setSpeedMultiplier(0.2), () -> Drive.getInstance().setSpeedMultiplier(1.0));
    }

    public static Command stop()
    {
        return Drive.getInstance().runOnce(() -> Drive.getInstance().stop());
    }

    public static Command setOdometer(Pose2d pose)
    {
        return Drive.getInstance().runOnce(() -> Drive.getInstance().setPose(pose));
    }
}
