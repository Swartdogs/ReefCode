package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants
{
    public static class AdvantageKit
    {
        public static final Mode SIM_MODE     = Mode.SIM;
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

        public static enum Mode
        {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }
}
