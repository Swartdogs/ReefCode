package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;

public class Vision extends SubsystemBase
{
    private final VisionIO                 _io;
    private final VisionIOInputsAutoLogged _inputs           = new VisionIOInputsAutoLogged();
    private final Drive                    _drive;
    private final PIDController            _rotatePID;
    private double                         _maxSpeed         = 0.3;
    private double                         _lastTimestamp    = 0.0;
    private boolean                        _targetingEnabled = false;

    public Vision(Drive drive, VisionIO io)
    {
        _drive     = drive;
        _io        = io;
        _rotatePID = new PIDController(0.026, 0, 0);
        _rotatePID.enableContinuousInput(-180, 180);
        _rotatePID.setTolerance(2);
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
                xyStdDev = Constants.Vision.VISION_STD_DEV_MULTI_XY 
                    + (avgDistance * Constants.Vision.VISION_DISTANCE_SCALE);
                thetaStdDev = Constants.Vision.VISION_STD_DEV_MULTI_THETA;
            }
            else
            {
                xyStdDev = Constants.Vision.VISION_STD_DEV_BASE_XY 
                    + (avgDistance * Constants.Vision.VISION_DISTANCE_SCALE);
                thetaStdDev = Constants.Vision.VISION_STD_DEV_BASE_THETA;
            }

            var stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

            _drive.addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp, stdDevs);
        }

        _lastTimestamp = _inputs.captureTimestamp;
    }

    public void setTargetingEnabled(boolean enabled)
    {
        _targetingEnabled = enabled;
    }
}
