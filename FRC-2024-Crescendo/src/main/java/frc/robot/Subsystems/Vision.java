package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public class Vision {

    PhotonCamera camera = new PhotonCamera("change-this");
    
    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.aprilField,651.25,323.25);
    /* tag id to location
    1,2 = left and right blue source
    3,4 = right and center red speaker
    5 = red amp
    6 = blue amp
    7,8 = center and right blue speaker
    9, 10 = right and left red source
    11, 12, 13 = left right center stage red
    14, 15, 16 = center left right stage blue
    */

    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0,0,0));


    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
