package frc.robot.Subsystems;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision {
    public final PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private double lastEstTimestamp = 0;
    public final PhotonPoseEstimator photonPoseEstimator;

    public Vision() {
        camera = new PhotonCamera("Front_Camera");

        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
            camera, 
            VisionConstants.kRobotToCam);

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout("2024-crescendo.json");
        } catch (IOException e) {
            e.printStackTrace();
            aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        }
    } 

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var visionEst = photonPoseEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }
    
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
}
