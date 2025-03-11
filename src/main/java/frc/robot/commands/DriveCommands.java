package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
        // @formatter:off
        return Commands.sequence
        (
            Commands.runOnce(() -> Drive.getInstance().rotateInit(setpoint.get(), maxSpeed)),
            joystickDrive(xSupplier, ySupplier, () -> Drive.getInstance().rotateExecute(), robotCentric, 2, 1)
        );
        // @formatter:on
    }

    public static Command resetGyro()
    {
        // @formatter:off
        return Commands.runOnce
        (
            () ->
            {
                var pose = Drive.getInstance().getPose();
                Drive.getInstance().setPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0)));
            }
        )
        .ignoringDisable(true);
        // @formatter:on
    }

    public static Command driveVolts(double volts)
    {
        return Commands.runOnce(() -> Drive.getInstance().runVolts(volts));
    }

    public static Command reduceSpeed()
    {
        // @formatter:off
        return Commands.startEnd
        (
            () -> Drive.getInstance().setSpeedMultiplier(0.2),
            () -> Drive.getInstance().setSpeedMultiplier(1.0)
        );
        // @formatter:on
    }

    public static Command stop()
    {
        return Drive.getInstance().runOnce(() -> Drive.getInstance().stop());
    }

    public static Command setOdometer(Pose2d pose)
    {
        return Drive.getInstance().runOnce(() -> Drive.getInstance().setPose(pose));
    }

    public static Command feedforwardCharacterization()
    {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples  = new LinkedList<>();
        Timer        timer           = new Timer();

        // @formatter:off
        return Commands.sequence
        (
            Commands.runOnce(() ->
            {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            Drive.getInstance().run(() ->
            {
                Drive.getInstance().runCharacterizationVolts(0.0);
            }).withTimeout(Constants.Drive.FF_START_DELAY),

            Commands.runOnce(timer::restart),

            Drive.getInstance().run(() ->
            {
                double voltage = timer.get() * Constants.Drive.FF_RAMP_RATE;
                Drive.getInstance().runCharacterizationVolts(voltage);
                velocitySamples.add(Drive.getInstance().getFFCharacterizationVelocity());
                voltageSamples.add((voltage));
            }).finallyDo(() ->
            {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;

                for (int i = 0; i < n; i++)
                {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }

                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            })
        );
        // @formatter:on
    }

    public static Command wheelRadiusCharacterization()
    {
        SlewRateLimiter                  limiter = new SlewRateLimiter(Constants.Drive.WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state   = new WheelRadiusCharacterizationState();

        // @formatter:off
        return Commands.parallel
        (
            Commands.sequence
            (
                Commands.runOnce(() -> limiter.reset(0.0)),

                Commands.run(
                    () ->
                    {
                        double speed = limiter.calculate(Constants.Drive.WHEEL_RADIUS_MAX_VELOCITY);
                        Drive.getInstance().runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    Drive.getInstance()
                )
            ),

            Commands.sequence
            (
                Commands.waitSeconds(1.0),

                Commands.runOnce(() ->
                {
                    state.positions = Drive.getInstance().getWheelRadiusCharacterizationPositions();
                    state.lastAngle = Drive.getInstance().getRotation();
                    state.gyroDelta = 0.0;
                }),

                Commands.run(() ->
                {
                    var rotation = Drive.getInstance().getRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                })
                .finallyDo(() ->
                {
                    double[] positions = Drive.getInstance().getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;

                    for (int i = 0; i < 4; i++)
                    {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }

                    double wheelRadius = (state.gyroDelta * Constants.Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000");
                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, " + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                })
            )
        );
        // @formatter:on
    }

    private static class WheelRadiusCharacterizationState
    {
        double[]   positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double     gyroDelta = 0.0;
    }
}
