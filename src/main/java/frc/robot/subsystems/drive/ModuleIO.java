package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO
{
    @AutoLog
    public static class ModuleIOInputs
    {
        public double     drivePositionRad       = 0.0;
        public double     driveVelocityRadPerSec = 0.0;
        public double     driveAppliedVolts      = 0.0;
        public double     driveCurrentAmps       = 0.0;
        public Rotation2d turnAbsolutePosition   = new Rotation2d();
        public Rotation2d turnPosition           = new Rotation2d();
        public double     turnVelocityRadPerSec  = 0.0;
        public double     turnAppliedVolts       = 0.0;
        public double     turnCurrentAmps        = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs)
    {
    }

    public default void setDriveVolts(double volts)
    {
    }

    public default void setTurnVolts(double volts)
    {
    }

    public default void setDriveBrakeMode(boolean enable)
    {
    }

    public default void setTurnBrakeMode(boolean enable)
    {
    }

    public default void setAngleOffset(Rotation2d offset)
    {
    }
}
