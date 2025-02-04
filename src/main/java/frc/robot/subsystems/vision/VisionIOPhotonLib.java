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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class VisionIOPhotonLib implements VisionIO
{
    private final PhotonCamera        _camera;
    private final AprilTagFieldLayout _fieldLayout;
    private final PhotonPoseEstimator _poseEstimator;
    private double                    _captureTimesStamp = 0.0;
    private Pose2d                    _pose              = new Pose2d();
    private boolean                   _hasPose           = false;
    private double[]                  _distances         = new double[] {};
    private int[]                     _targetIds         = new int[] {};
    private double[]                  _targetYaws        = new double[] {};
    private int                       _numProcessedTargets;

    public VisionIOPhotonLib(Drive drive)
    {
        _camera = new PhotonCamera(Constants.Vision.PHOTON_CAMERA_NAME);

        try
        {
            // Load the 2024 field layout
            _fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        }
        catch (Exception e)
        {
            System.err.println("Failed to load AprilTag layout: " + e.getMessage());
            throw new RuntimeException(e);
        }

        _poseEstimator = new PhotonPoseEstimator(_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.ROBOT_TO_CAMERA);
        _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        NetworkTableInstance.getDefault().addListener(NetworkTableInstance.getDefault().getEntry("/photonvision/" + Constants.Vision.PHOTON_CAMERA_NAME + "/latencyMillis"), EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event ->
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
            List<PhotonTrackedTarget> processedTargets = new ArrayList<>();

            // Process all targets
            for (PhotonTrackedTarget target : targets)
            {
                ids.add(target.getFiducialId());
                yaws.add(target.getYaw());

                var transform = target.getBestCameraToTarget();
                var distance  = Math.hypot(transform.getX(), transform.getY());
                distances.add(distance);

                if (distance <= Constants.Vision.MAX_DETECTION_RANGE)
                {
                    processedTargets.add(target);
                }
            }

            // Get pose estimate from processed targets
            _poseEstimator.setReferencePose(drive.getPose());
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
                _distances           = distances.stream().mapToDouble(Double::doubleValue).toArray();
                _pose                = pose;
                _numProcessedTargets = processedTargets.size();
                _hasPose             = hasPose;
                _targetIds           = ids.stream().mapToInt(Integer::intValue).toArray();
                _targetYaws          = yaws.stream().mapToDouble(Double::doubleValue).toArray();
            }
        });
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result)
    {
        _poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return _poseEstimator.update(result);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs)
    {
        var results = _camera.getAllUnreadResults();
        if (results.isEmpty())
        {
            // No results available
            inputs.hasTargets      = false;
            inputs.numTargets      = 0;
            inputs.targetIds       = new int[0];
            inputs.targetYaws      = new double[0];
            inputs.targetPitches   = new double[0];
            inputs.targetAreas     = new double[0];
            inputs.targetDistances = new double[0];
            inputs.hasPose         = false;
            return;
        }

        var result = results.get(results.size() - 1);

        inputs.captureTimestamp = result.getTimestampSeconds();
        inputs.hasTargets       = result.hasTargets();

        if (inputs.hasTargets)
        {
            List<PhotonTrackedTarget> targets    = result.getTargets();
            int                       numTargets = targets.size();

            inputs.targetIds       = new int[numTargets];
            inputs.targetYaws      = new double[numTargets];
            inputs.targetPitches   = new double[numTargets];
            inputs.targetAreas     = new double[numTargets];
            inputs.targetDistances = new double[numTargets];

            for (int i = 0; i < numTargets; i++)
            {
                PhotonTrackedTarget target = targets.get(i);
                inputs.targetIds[i]     = target.getFiducialId();
                inputs.targetYaws[i]    = target.getYaw();
                inputs.targetPitches[i] = target.getPitch();
                inputs.targetAreas[i]   = target.getArea();

                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                inputs.targetDistances[i] = Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY());
            }

            inputs.numTargets = numTargets;

            // Update pose estimate if we have new data
            if (_captureTimesStamp != inputs.captureTimestamp)
            {
                Optional<EstimatedRobotPose> poseEstimate = getEstimatedGlobalPose(_pose, result);
                if (poseEstimate.isPresent())
                {
                    inputs.pose    = poseEstimate.get().estimatedPose.toPose2d();
                    inputs.hasPose = true;
                    _pose          = inputs.pose;
                }
                else
                {
                    inputs.hasPose = false;
                }
                _captureTimesStamp = inputs.captureTimestamp;
            }
        }
        else
        {
            // No targets visible
            inputs.numTargets      = 0;
            inputs.targetIds       = new int[0];
            inputs.targetYaws      = new double[0];
            inputs.targetPitches   = new double[0];
            inputs.targetAreas     = new double[0];
            inputs.targetDistances = new double[0];
            inputs.hasPose         = false;
        }
    }
}
