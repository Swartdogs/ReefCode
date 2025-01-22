package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

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
        _drive = drive;
        _io    = io;

        _rotatePID = new PIDController(0.026, 0, 0);
        _rotatePID.enableContinuousInput(-180, 180);
        _rotatePID.setTolerance(2);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Vision", _inputs);

        // if (Constants.Vision.ENABLE_POSE_CORRECTION)
        // {
        // if (!DriverStation.isAutonomous() && _lastTimestamp !=
        // _inputs.captureTimestamp)
        // {
        // if (_inputs.hasPose != false)
        // {
        // _drive.addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp);
        // }

        // _lastTimestamp = _inputs.captureTimestamp;
        // }
        // }
    }

    public void setTargetingEnabled(boolean enabled)
    {
        _targetingEnabled = enabled;
    }
}
