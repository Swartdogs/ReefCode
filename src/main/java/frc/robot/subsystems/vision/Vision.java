package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
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
    private final VisionIOInputsAutoLogged _inputs   = new VisionIOInputsAutoLogged();
    private final Camera                   _camera;
    private int                            _pidTagId = 0;
    private Pose2d                         _lastPose = new Pose2d();

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
        }
    }

    private int getTagIndex(int id)
    {
        return Arrays.stream(_inputs.targetIds).boxed().toList().indexOf(id);
    }

    public boolean hasTarget()
    {
        return _pidTagId != 0 ? hasTarget(_pidTagId) : false;
    }

    public boolean hasTarget(int id)
    {
        return getTagIndex(id) != -1;
    }

    public Pose2d getEstimatedPose()
    {
        return _inputs.pose;
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

    public void setVisionReference(int id)
    {
        _pidTagId = id;
    }
}
