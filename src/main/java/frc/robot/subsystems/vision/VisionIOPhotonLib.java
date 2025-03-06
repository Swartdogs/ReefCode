package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision.Camera;

public class VisionIOPhotonLib implements VisionIO
{
    private final PhotonCamera        _camera;
    private final PhotonPoseEstimator _poseEstimator;
    private final Camera              _cameraSettings;
    private double                    _captureTimesStamp = 0.0;
    private Pose2d                    _pose              = new Pose2d();
    private boolean                   _hasPose           = false;
    private double[]                  _distances         = new double[] {};
    private int[]                     _targetIds         = new int[] {};
    private double[]                  _targetYaws        = new double[] {};
    private double[]                  _targetPitches     = new double[] {};
    private double[]                  _targetAreas       = new double[] {};
    private int                       _numProcessedTargets;
    private double[]                  _cornerX           = new double[] {};
    private double[]                  _cornerY           = new double[] {};

    public VisionIOPhotonLib(Camera cameraSettings)
    {
        _cameraSettings = cameraSettings;
        _camera         = new PhotonCamera(_cameraSettings.cameraName);

        _poseEstimator = new PhotonPoseEstimator(Constants.Field.APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _cameraSettings.robotToCamera);
        _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        NetworkTableInstance.getDefault().addListener(NetworkTableInstance.getDefault().getEntry("/photonvision/" + _cameraSettings.cameraName + "/latencyMillis"), EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event ->
        {
            var results = _camera.getAllUnreadResults();
            if (results.isEmpty())
            {
                return;
            }
            // get the latest result (last in the list)
            var result = results.get(results.size() - 1);

            double                    timestamp = result.getTimestampSeconds();
            List<PhotonTrackedTarget> targets   = result.getTargets();

            List<Double>              distances        = new ArrayList<>();
            List<Integer>             ids              = new ArrayList<>();
            List<Double>              yaws             = new ArrayList<>();
            List<Double>              pitches          = new ArrayList<>();
            List<Double>              areas            = new ArrayList<>();
            List<PhotonTrackedTarget> processedTargets = new ArrayList<>();
            List<Double>              cornerXList      = new ArrayList<>();
            List<Double>              cornerYList      = new ArrayList<>();

            // Process all targets
            for (PhotonTrackedTarget target : targets)
            {
                ids.add(target.getFiducialId());
                yaws.add(target.getYaw());
                pitches.add(target.getPitch());
                areas.add(target.getArea());

                for (TargetCorner corner : target.getDetectedCorners())
                {
                    cornerXList.add(corner.x);
                    cornerYList.add(corner.y);
                }

                var transform = target.getBestCameraToTarget();
                var distance  = Math.hypot(transform.getX(), transform.getY());
                distances.add(distance);

                if (distance <= Constants.Vision.MAX_DETECTION_RANGE)
                {
                    processedTargets.add(target);
                }
            }

            // Get pose estimate from processed targets
            _poseEstimator.setReferencePose(Drive.getInstance().getPose());
            Optional<EstimatedRobotPose> estimatedPose = _poseEstimator.update(result);
            Pose2d                       pose          = new Pose2d();
            boolean                      hasPose       = false;

            if (estimatedPose.isPresent())
            {
                pose    = estimatedPose.get().estimatedPose.toPose2d();
                hasPose = true;
            }

            synchronized (VisionIOPhotonLib.this)
            {
                _captureTimesStamp   = timestamp;
                _pose                = pose;
                _hasPose             = hasPose;
                _numProcessedTargets = processedTargets.size();
                _distances           = distances.stream().mapToDouble(Double::doubleValue).toArray();
                _targetIds           = ids.stream().mapToInt(Integer::intValue).toArray();
                _targetYaws          = yaws.stream().mapToDouble(Double::doubleValue).toArray();
                _targetPitches       = pitches.stream().mapToDouble(Double::doubleValue).toArray();
                _targetAreas         = areas.stream().mapToDouble(Double::doubleValue).toArray();
                _cornerX             = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
                _cornerY             = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
            }
        });
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs)
    {
        inputs.captureTimestamp = _captureTimesStamp;
        inputs.pose             = _pose;
        inputs.hasPose          = _hasPose;
        inputs.numTargets       = _numProcessedTargets;
        inputs.targetDistances  = _distances;
        inputs.targetIds        = _targetIds;
        inputs.targetYaws       = _targetYaws;
        inputs.targetPitches    = _targetPitches;
        inputs.targetAreas      = _targetAreas;
        inputs.cornerX          = _cornerX;
        inputs.cornerY          = _cornerY;
    }
}
