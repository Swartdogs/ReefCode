package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO
{
    @AutoLog // adds logging to ManipulatorIO interface
    public static class ManipulatorIOInputs
    {
        public double  topAppliedVolts    = 0.0;
        public double  topCurrentAmps     = 0.0;
        public double  bottomAppliedVolts = 0.0;
        public double  bottomCurrentAmps  = 0.0;
        public boolean hasCoral           = false;
    }

    public default void updateInputs(ManipulatorIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }
}
