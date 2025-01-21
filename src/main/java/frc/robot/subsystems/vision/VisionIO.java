package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO
{
    @AutoLog
    public static class VisionIOInputs
    {
        public double   captureTimestamp    = 0.0;
        public double[] tagX             = new double[] {}; //represents the x distance of all tags
        //public double[] tagY             = new double[] {};
        public double[] closeTagX        = new double[] {};
        //public double[] closeTagY        = new double[] {};
        public Pose2d   pose                = new Pose2d();
        public boolean  hasPose             = false;
        public int      numProcessedTargets = 0;
        public double[] targetDistances     = new double[] {}; //Target = specific april tag
        public int[]    targetIds           = new int[] {};
        public double[] targetYaws          = new double[] {};
    }

    public default void updateInputs(VisionIOInputs inputs)
    {
    }
}