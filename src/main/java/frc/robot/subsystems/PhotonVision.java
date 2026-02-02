/*import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision {
	
	
	List<PhotonCamera> robotCameras;

	public PhotonVision (List<PhotonCamera> cameras) {
    List <PhotonCamera> robotCameras = cameras;
    List<PhotonPoseEstimator> poseEstimators;
   
    // Make a list of PhotonPoseEstimators
    for i in robotCameras {
    	camera = new PhotonCamera(Constants.vision.localizationCameraName[i])
    	cameraEst.append(new PhotonPoseEstimator(Constants.vision.kTagLayout,
    		                        PoseStratagy MULTI_TAG_PNP_ON_COPROCESSOR,
    		                        Constants.vision.localizationCameraToRobot[i]));
    	// TODO: tell the programming team to make that var
    	// actually, tell them to make both
   
    	cameraEst.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
    	// TODO: learn what 'PoseStratagy' is
    }
	}

	public Optional<EstimatedRobotPose> getPose(String cameraName, var results) {
		// check to see which camera matches the desired name, and when it's found,
		// return the pose
		visionEst = Optional.empty();
		for i in robotCameras {
			if (i.getName() == cameraName) {
			visionEst = i.update(i.getLatestResult());
			break;
			// there better not be multiple cameras of the same name
			// if so, I'm gonna have a talk with someone! /hj
			}
		}

		if (visionEst != Optional.empty()) {
			return visionEst;
		}
		else {
				pass;
				// TODO: write code to indicate something has gone seriously wrong
				// we'll probably notice anyway but whatever
			}
	}

	
}
*/