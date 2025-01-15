package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO
{
    @AutoLog
    public static class ElevatorIOInputs
    {
        public double   extensionPosition = 0.0;
        public double   extensionVelocity = 0.0;
        public double   leaderVolts       = 0.0;
        public double leaderCurrent     = 0.0;
        public double   followerVolts     = 0.0;
        public double followerCurrent   = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
