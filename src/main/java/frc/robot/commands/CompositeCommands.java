package frc.robot.commands;

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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;

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
            double     clamp           = MathUtil
                    .clamp((Constants.Drive.SPEED_ELEVATOR_M * Elevator.getInstance().getExtension() + Constants.Drive.SPEED_ELEVATOR_B), Constants.Drive.MIN_SPEED_ELEVATOR_MULTIPLIER, Constants.Drive.MAX_SPEED_ELEVATOR_MULTIPLIER);

            // Square values
            linearMagnitude = Math.pow(linearMagnitude, translateExponent);
            omega           = Math.copySign(Math.pow(Math.abs(omega), rotateExponent), omega);

            // Calculate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Convert to field relative speeds & send command
            if (robotCentric.getAsBoolean())
            {
                var chassisSpeeds = new ChassisSpeeds(linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED * clamp, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED * clamp, omega * Constants.Drive.MAX_ANGULAR_SPEED * clamp);

                Drive.getInstance().runVelocity(chassisSpeeds);
            }
            else
            {
                var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED * clamp, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED * clamp, omega * Constants.Drive.MAX_ANGULAR_SPEED * clamp, Drive.getInstance().getRotation()
                );

                Drive.getInstance().runVelocity(chassisSpeeds);
            }
        }, Drive.getInstance());
    }

    public static Command output()
    {
        return Commands.sequence(ManipulatorCommands.output(), Commands.waitSeconds(Constants.Elevator.WAIT_TIME), ElevatorCommands.setHeight(ElevatorHeight.Stow)).unless(() -> !Manipulator.getInstance().detectedCoral());
    }

    public static Command setHeight(ElevatorHeight height)
    {
        return Commands.sequence(ElevatorCommands.setHeight(height), Commands.waitUntil(() -> Elevator.getInstance().atSetpoint()));
    }
}
