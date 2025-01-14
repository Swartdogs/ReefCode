package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import frc.robot.Constants;

public class VisionIOPhotonLib implements VisionIO
{
    private final PhotonCamera _camera = new PhotonCamera(Constants.Vision.PHOTON_CAMERA_NAME);
    
}
