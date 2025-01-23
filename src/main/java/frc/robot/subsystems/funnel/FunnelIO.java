package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO
{
    @AutoLog
    public static class FunnelIOInputs
    {
        public boolean isDropped = false;
    }

    public default void updateInputs(FunnelIOInputs inputs)
    {
    }

    public default void setState(boolean state)
    {
    }
}
