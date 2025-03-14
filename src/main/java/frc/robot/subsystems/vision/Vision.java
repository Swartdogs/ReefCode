package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase
{
    public enum Camera
    {
        Front("front", new Transform3d(new Translation3d(Units.inchesToMeters(9), Units.inchesToMeters(12), Units.inchesToMeters(11.75)), new Rotation3d(Rotation2d.fromDegrees(-45)))),
        Back("back", new Transform3d(new Translation3d(Units.inchesToMeters(1.25), Units.inchesToMeters(5.5), Units.inchesToMeters(35)), new Rotation3d(Rotation2d.fromDegrees(180))));

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
    private final VisionIOInputsAutoLogged _inputs           = new VisionIOInputsAutoLogged();
    private final Camera                   _camera;
    private final PIDController            _xDriveController = new PIDController(Constants.Vision.DRIVE_KP, 0, Constants.Vision.DRIVE_KD); // forward and back
    private final PIDController            _yDriveController = new PIDController(Constants.Vision.DRIVE_KP, 0, Constants.Vision.DRIVE_KD); // left to right
    private Translation2d                  _reference        = new Translation2d();
    private int                            _pidTagId         = 0;
    private Pose2d                         _lastPose         = new Pose2d();

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

            var    stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
            double alpha   = 0.85;

            double smoothedX     = alpha * _inputs.pose.getX() + (1 - alpha) * _lastPose.getX();
            double smoothedY     = alpha * _inputs.pose.getY() + (1 - alpha) * _lastPose.getY();
            double smoothedTheta = alpha * _inputs.pose.getRotation().getRadians() + (1 - alpha) * _lastPose.getRotation().getRadians();

            Pose2d smoothedPose = new Pose2d(smoothedX, smoothedY, new Rotation2d(smoothedTheta));

            _lastPose = smoothedPose;

            Drive.getInstance().addVisionMeasurement(smoothedPose, _inputs.captureTimestamp, stdDevs);

            if (_pidTagId != 0)
            {
                Logger.recordOutput("Vision/Theta", getAngleOffset());
                Logger.recordOutput("Vision/Distance", getTargetDistance(_pidTagId));
                Logger.recordOutput("Vision/Gx", getXOffset());
                Logger.recordOutput("Vision/Gy", getYOffset());
            }
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

        if (index != -1)
        {
            return _inputs.targetDistances[index];
        }
        else
        {
            return -1;
        }
    }

    public Rotation2d getTargetYaw(int id)
    {
        int index = getTagIndex(id);

        if (index != -1)
        {
            return _inputs.targetYaws[index];
        }
        else
        {
            return new Rotation2d();
        }
    }

    public void setXDriveSetpoint(int id, double distanceOffset)
    {
        _pidTagId  = id;
        _reference = new Translation2d(distanceOffset, _reference.getY());
        _xDriveController.setSetpoint(distanceOffset);
    }

    public void setYDriveSetpoint(int id, double distanceOffset)
    {
        _pidTagId  = id;
        _reference = new Translation2d(_reference.getX(), distanceOffset);
        _yDriveController.setSetpoint(distanceOffset);
    }

    public void setVisionReference(int id, Translation2d reference)
    {
        _pidTagId  = id;
        _reference = reference;

        _xDriveController.setSetpoint(reference.getX());
        _yDriveController.setSetpoint(reference.getY());
    }

    public Rotation2d getAngleToTag()
    {
        if (hasTarget(_pidTagId))
        {
            return Drive.getInstance().getRotation().minus(Constants.Field.getTagAngle(_pidTagId));
        }
        else
        {
            return new Rotation2d();
        }
    }

    private Rotation2d getAngleOffset()
    {
        Rotation2d yaw      = getTargetYaw(_pidTagId);
        Rotation2d cToO     = _camera.robotToCamera.getRotation().toRotation2d();
        Rotation2d rotation = Drive.getInstance().getRotation();
        Rotation2d tagAngle = Constants.Field.getTagAngle(_pidTagId);

        Logger.recordOutput("Vision/yaw", yaw);
        Logger.recordOutput("Vision/cToO", cToO);
        Logger.recordOutput("Vision/rotation", rotation);
        Logger.recordOutput("Vision/tagAngle", tagAngle);

        return (yaw.plus(cToO)).minus(rotation.minus(tagAngle));
    }

    private double getXOffset()
    {
        return getTargetDistance(_pidTagId) * Math.cos(getAngleOffset().getRadians()) + _reference.getX();
    }

    private double getYOffset()
    {
        return getTargetDistance(_pidTagId) * Math.sin(getAngleOffset().getRadians()) + _reference.getY();
    }

    public double getXDistanceCalculation()
    {
        if (!hasTarget(_pidTagId))
        {
            return 0;
        }
        else
        {
            return -_xDriveController.calculate(getXOffset());
        }
    }

    public double getYDistanceCalculation()
    {
        if (!hasTarget(_pidTagId))
        {
            return 0;
        }
        else
        {
            return -_yDriveController.calculate(getYOffset());
        }
    }

    public double getCommonDifference()
    {
        double x      = Math.abs(getXDistanceCalculation());
        double y      = Math.abs(getYDistanceCalculation());
        double excess = 0.0;

        if (x > y)
        {
            excess = x - y;
        }
        else if (y > x)
        {
            excess = y - x;
        }

        return excess;
    }

    public double getXDMod()
    {
        double x = getXDistanceCalculation();
        double y = getYDistanceCalculation();

        if (Math.abs(x) > Math.abs(y))
        {
            x = Math.copySign(y, x);
        }

        return x;
    }

    public double getYMod()
    {
        double x = getXDistanceCalculation();
        double y = getYDistanceCalculation();

        if (Math.abs(y) > Math.abs(x))
        {
            y = Math.copySign(x, y);
        }

        return y;
    }
}
