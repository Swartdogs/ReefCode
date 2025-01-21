package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class VisionIOPhotonLib implements VisionIO
{
    private final PhotonCamera _camera = new PhotonCamera(Constants.Vision.PHOTON_CAMERA_NAME);
    private double _captureTimesStamp = 0.0;
    private double[] _tagX = new double[] {};
    private double[] _closeTagX = new double[] {};
    private Pose2d _pose = new Pose2d();
    private boolean _hasPose = false;
    private int[] _targetIds = new int[] {};
    private double[] _targetYaws = new double[] {};
    private int _numProcessedTargets ;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator _PoseEstimator;
    private final Transform3d _robotToCamera;
    private final Pose3d _lastEstimatedPose = new Pose3d();
    
    public class VisionIOPhotonlib(Drive drive)
    {
        try
        {
            fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); //TODO: update layout
        }
        catch(Exception e)
        {
            System.out.println("Exception encountered: " + e.getMessage());
        }
        _PoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _robotToCamera);
        _PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    @Override
    public synchronized void updateInputs(VisionIOInputs inputs)
    {
        PhotonPipelineResult _result = _camera.getLatestResult();
        inputs.captureTimestamp = _captureTimesStamp;
        inputs.tagX = _tagX;
        inputs.closeTagX = _closeTagX;
        inputs.pose = _pose;
        inputs.hasPose = _hasPose;
        inputs.targetIds = _targetIds;
        inputs.targetYaws = _targetYaws;
        inputs.numProcessedTargets = _numProcessedTargets;
        
        if (_result.hasTargets())
        {
            Optional<EstimatedRobotPose> _poseOptional = _PoseEstimator.update(_result);
            if (_poseOptional.isPresent())
            {
                EstimatedRobotPose _estimatedPose = _poseOptional.get();
                inputs.captureTimestamp = _result.getTimestampSeconds();
                inputs._estimatedPose
            }
        }
    }
}
