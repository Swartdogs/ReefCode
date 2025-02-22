package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO
{
    @AutoLog // adds logging to ManipulatorIO interface
    public static class ManipulatorIOInputs
    {
        public double  leftAppliedVolts   = 0.0;
        public double  leftCurrentAmps    = 0.0;
        public double  rightAppliedVolts  = 0.0;
        public double  rightCurrentAmps   = 0.0;
        public boolean startSensorTripped = false;
        public boolean endSensorTripped   = false;
    }

    public default void updateInputs(ManipulatorIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
    }

    public default void setLeftVolts(double volts)
    {

    }
}
