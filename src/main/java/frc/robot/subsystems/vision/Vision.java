package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase
{
    public enum Camera
    {
        Front("front", new Transform3d(new Translation3d(), new Rotation3d())), Back("back", new Transform3d(new Translation3d(), new Rotation3d()));

        public final String      cameraName;
        public final Transform3d robotToCamera;

        private Camera(String cameraName, Transform3d robotToCamera)
        {
            this.cameraName    = cameraName;
            this.robotToCamera = robotToCamera;
        }
    }

    private static final Map<Camera, Vision> _instances = new HashMap<>();

    public static Vision getInstance(Camera camera)
    {
        if (!_instances.containsKey(camera))
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL, SIM -> new VisionIOPhotonLib(camera);
                default -> new VisionIO() {};
            };

            _instances.put(camera, new Vision(io, camera));
        }
        return _instances.get(camera);
    }

    private final VisionIO                 _io;
    private final VisionIOInputsAutoLogged _inputs = new VisionIOInputsAutoLogged();
    private final Camera                   _camera;

    private final PIDController _anglePIDController  = new PIDController(Constants.Vision.TURN_KP, 0, Constants.Vision.TURN_KD);
    private final PIDController _xDriveController = new PIDController(Constants.Vision.DRIVE_KP, 0, Constants.Vision.DRIVE_KD); //forward and back
    private final PIDController _yDriveController = new PIDController(Constants.Vision.DRIVE_KP, 0, Constants.Vision.DRIVE_KD); //left to right

    private int _pidTagId = 0;

    private Vision(VisionIO io, Camera camera)
    {
        _io     = io;
        _camera = camera;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Vision/" + _camera.toString(), _inputs);

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

            Drive.getInstance().addVisionMeasurement(_inputs.pose, _inputs.captureTimestamp, stdDevs);
        }
    }

    private int getTagIndex(int id)
    {
        return Arrays.stream(_inputs.targetIds).boxed().toList().indexOf(id);
    }

    public boolean hasTarget(int id)
    { 
        return getTagIndex(id) != -1;
    }

    public double getTargetDistance(int id)
    {
        int index = getTagIndex(id);

        if(index != -1)
        {
            return _inputs.targetDistances[index];
        }
        else
        {
            return -1;
        }
    }

    public double getTargetYaw(int id)
    {
        int index = getTagIndex(id);

        if(index != -1)
        {
            return _inputs.targetYaws[index];
        }
        else
        {
            return -1;
        }
    }

    public void setAngleSetpoint(int id, double angleOffset)
    {
        _pidTagId = id;
        _anglePIDController.setSetpoint(angleOffset);
    }

    public void setXDriveSetpoint(int id, double distanceOffset)
    {
        _pidTagId = id;
        _xDriveController.setSetpoint(distanceOffset);
    }

    public void setYDriveSetpoint(int id, double distanceOffset)
    {
        _pidTagId = id;
        _yDriveController.setSetpoint(distanceOffset);
    }

    public void setVisionReference(int id, Pose2d reference)
    {
        _pidTagId = id;
        
        _anglePIDController.setSetpoint(reference.getRotation().getDegrees());
        _xDriveController.setSetpoint(reference.getX());
        _yDriveController.setSetpoint(reference.getY());
    }

    public double getAngleCalculation()
    {
        if(!hasTarget(_pidTagId))
        {
            return 0;
        }
        else
        {
            return _anglePIDController.calculate(getTargetYaw(_pidTagId));
        }
    }

    public double getXDistanceCalculation()
    {
        if(!hasTarget(_pidTagId))
        {
            return 0;
        }
        else
        {
            return _xDriveController.calculate(getTargetDistance(_pidTagId));
        }
    }

    public double getYDistanceCalculation()
    {
        if(!hasTarget(_pidTagId))
        {
            return 0;
        }
        else
        {
            return _yDriveController.calculate(getTargetDistance(_pidTagId));
        }
    }
}
