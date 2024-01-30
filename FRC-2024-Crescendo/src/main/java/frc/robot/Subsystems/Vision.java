package frc.robot.Subsystems;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase{
    public final PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private double lastEstTimestamp = 0;
    public final PhotonPoseEstimator photonPoseEstimator;

    public Vision() {
        camera = new PhotonCamera(VisionConstants.kFrontCameraName);
        aprilTagFieldLayout = VisionConstants.k2024CrescendoTagField;
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
            camera, 
            VisionConstants.kRobotToCam);

        // try {
        //     aprilTagFieldLayout = new AprilTagFieldLayout("2024-crescendo.json");
        // } catch (IOException e) {
        //     e.printStackTrace();
        //     aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        // }
    } 

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var visionEst = photonPoseEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    @Override
    public void periodic(){
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d bestCamToTarget = target.getBestCameraToTarget();

            SmartDashboard.putNumber("aprilTranslation",bestCamToTarget.getX());
            SmartDashboard.putNumber("aprilStrafe",bestCamToTarget.getY());
            SmartDashboard.putNumber("aprilRotation",180-Math.toDegrees(bestCamToTarget.getRotation().getZ()));
        }
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
