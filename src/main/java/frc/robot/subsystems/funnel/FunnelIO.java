package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO
{
    @AutoLog
    public static class FunnelIOInputs
    {
        public double funnelVolts = 0.0;
    }

    public default void updateInputs(FunnelIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
