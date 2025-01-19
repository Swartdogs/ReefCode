package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO
{
    @AutoLog
    public static class GyroIOInputs
    {
        public boolean      connected               = false;
        public Rotation2d   yawPosition             = new Rotation2d();
        public double       yawVelocityRadPerSec    = 0.0;
        public Rotation2d   pitchPosition           = new Rotation2d();
        public Rotation2d   rollPosition            = new Rotation2d();
        public double[]     odometryYawTimestamps   = new double[] {};
        public Rotation2d[] odometryYawPositions    = new Rotation2d[] {};
        public double[]     odometryPitchTimestamps = new double[] {};
        public Rotation2d[] odometryPitchPositions  = new Rotation2d[] {};
        public double[]     odometryRollTimestamps  = new double[] {};
        public Rotation2d[] odometryRollPositions   = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs)
    {
    }
}
