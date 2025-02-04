package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO
{
    @AutoLog
    public static class VisionIOInputs
    {
        public double   captureTimestamp = 0.0;
        public Pose2d   pose             = new Pose2d();
        public boolean  hasTargets       = false;
        public boolean  hasPose          = false;
        public int      numTargets       = 0;
        public double[] targetDistances  = new double[] {}; // Target = specific april tag
        public int[]    targetIds        = new int[] {};
        public double[] targetYaws       = new double[] {};
        public double[] targetPitches    = new double[] {};
        public double[] targetAreas      = new double[] {};
    }

    public default void updateInputs(VisionIOInputs inputs)
    {
    }
}
