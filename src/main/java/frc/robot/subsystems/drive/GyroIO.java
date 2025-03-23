package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO
{
    @AutoLog
    public static class GyroIOInputs
    {
        public Rotation2d yawPosition          = new Rotation2d();
        public double     yawVelocityRadPerSec = 0.0;
        public Rotation2d pitchPosition        = new Rotation2d();
        public Rotation2d rollPosition         = new Rotation2d();
        public double     accelerationX        = 0;
        public double     accelerationY        = 0;
        public double     accelerationZ        = 0;
    }

    public default void updateInputs(GyroIOInputs inputs)
    {
    }
}
