package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;

public class Vision extends SubsystemBase
{
    private final VisionIO                 _io;
    private final VisionIOInputsAutoLogged _inputs           = new VisionIOInputsAutoLogged();
    private final Drive                    _drive;

    public Vision(Drive drive, VisionIO io)
    {
        _drive     = drive;
        _io        = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Vision", _inputs);

        if (_inputs.hasPose)
        {
            double avgDistance = 0;
            if (_inputs.targetDistances.length > 0)
            {
                avgDistance = Arrays.stream(_inputs.targetDistances).average().getAsDouble();
            }

            double xyStdDev;
            double thetaStdDev;

            if (_inputs.numTargets > 1)
            {
                xyStdDev    = Constants.Vision.VISION_STD_DEV_MULTI_XY + (avgDistance * Constants.Vision.VISION_DISTANCE_SCALE);
                thetaStdDev = Constants.Vision.VISION_STD_DEV_MULTI_THETA;
            }
            else
            {
                xyStdDev    = Constants.Vision.VISION_STD_DEV_BASE_XY + (avgDistance * Constants.Vision.VISION_DISTANCE_SCALE);
                thetaStdDev = Constants.Vision.VISION_STD_DEV_BASE_THETA;
            }

            var stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

            _drive.addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp, stdDevs);
        }
    }
}