package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO
{
    @AutoLog
    public static class ModuleIOInputs
    {
        public double   extensionPosition = 0.0;
        public double   extensionVelocity = 0.0;
        public double   leaderVolts       = 0.0;
        public double[] leaderCurrent     = new double[] {};
        public double   followerVolts     = 0.0;
        public double[] followerCurrent   = new double[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
