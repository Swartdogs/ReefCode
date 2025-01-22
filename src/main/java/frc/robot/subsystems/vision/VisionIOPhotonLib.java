package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class VisionIOPhotonLib implements VisionIO
{
    private final PhotonCamera  _camera            = new PhotonCamera(Constants.Vision.PHOTON_CAMERA_NAME);
    private double              _captureTimesStamp = 0.0;
    private double[]            _tagX              = new double[] {};
    private double[]            _closeTagX         = new double[] {};
    private Pose2d              _pose              = new Pose2d();
    private boolean             _hasPose           = false;
    private double[]            _distances         = new double[] {};
    private int[]               _targetIds         = new int[] {};
    private double[]            _targetYaws        = new double[] {};
    private int                 _numProcessedTargets;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator _poseEstimator;
    private Transform3d         _robotToCamera;
    private final Pose3d        _lastEstimatedPose = new Pose3d();

    public void VisionIOPhotonlib(Drive drive)
    {
        try
        {
            fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        }
        catch (Exception e)
        {
            System.out.println("Exception encountered: " + e.getMessage());
        }

        _poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _robotToCamera);
        // _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        NetworkTableInstance.getDefault().addListener(NetworkTableInstance.getDefault().getEntry("/photonvision/" + Constants.Vision.PHOTON_CAMERA_NAME + "/latencyMillis"), EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event ->
        {
            PhotonPipelineResult      result           = _camera.getLatestResult();
            double                    timestamp        = result.getTimestampSeconds();
            List<Double>              tagXList         = new ArrayList<>();
            List<Double>              closeTagXList    = new ArrayList<>();
            List<PhotonTrackedTarget> processedTargets = new ArrayList<>();
            List<Double>              distances        = new ArrayList<>();
            List<Integer>             ids              = new ArrayList<>();
            List<Double>              yaws             = new ArrayList<>();

            for (PhotonTrackedTarget target : result.getTargets())
            {
                ids.add(target.getFiducialId());
                yaws.add(target.getYaw());

                for (TargetCorner corner : target.getDetectedCorners())
                {
                    tagXList.add(corner.x);
                }

                var transform = target.getBestCameraToTarget();
                var distance  = Math.hypot(transform.getX(), transform.getY());
                distances.add(distance);

                if (distance <= Constants.Vision.MAX_DETECTION_RANGE)
                {
                    processedTargets.add(target);

                    for (TargetCorner corner : target.getDetectedCorners())
                    {
                        closeTagXList.add(corner.x);
                    }

                }
            }

            PhotonPipelineResult processedResult = new PhotonPipelineResult();
            processedResult.getTimestampSeconds(); // TODO: see if need change

            Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(drive.getPose(), processedResult);
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
                _tagX                = tagXList.stream().mapToDouble(Double::doubleValue).toArray();
                _closeTagX           = closeTagXList.stream().mapToDouble(Double::doubleValue).toArray();
                _distances           = distances.stream().mapToDouble(Double::doubleValue).toArray();
                _pose                = pose;
                _numProcessedTargets = processedTargets.size();
                _hasPose             = hasPose;
                _targetIds           = ids.stream().mapToInt(Integer::intValue).toArray();
                _targetYaws          = yaws.stream().mapToDouble(Double::doubleValue).toArray();
            }
        });
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs)
    {
        // PhotonPipelineResult _result = _camera.getLatestResult();
        inputs.captureTimestamp    = _captureTimesStamp;
        inputs.tagX                = _tagX;
        inputs.closeTagX           = _closeTagX;
        inputs.pose                = _pose;
        inputs.hasPose             = _hasPose;
        inputs.targetIds           = _targetIds;
        inputs.targetYaws          = _targetYaws;
        inputs.numProcessedTargets = _numProcessedTargets;

        // if (_result.hasTargets())
        // {
        // Optional<EstimatedRobotPose> _poseOptional = _PoseEstimator.update(_result);
        // if (_poseOptional.isPresent())
        // {
        // EstimatedRobotPose _estimatedPose = _poseOptional.get();
        // inputs.captureTimestamp = _result.getTimestampSeconds();
        // inputs._estimatedPose
        // }
        // }
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result)
    {
        _poseEstimator.setLastPose(prevEstimatedRobotPose);
        return _poseEstimator.update(result);
    }
}
