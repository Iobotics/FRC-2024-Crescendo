package frc.robot.Subsystems;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class Vision extends SubsystemBase{
    public final PhotonCamera swifferCamera;
    public final PhotonCamera intakeCamera;
    public boolean intakeCameraFree;

    private AprilTagFieldLayout aprilTagFieldLayout;
    public final PhotonPoseEstimator swifferPoseEstimator;
    public final PhotonPoseEstimator intakePoseEstimator;
    // private final Field2d m_field = new Field2d();
    private Swerve swerve;
    @SuppressWarnings("unused")
    private EstimatedRobotPose latestEstimatedPose;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        this.swifferCamera = new PhotonCamera(VisionConstants.kFrontCameraName);
        this.intakeCamera = new PhotonCamera(VisionConstants.kIntakeCameraName);
        aprilTagFieldLayout = VisionConstants.k2024CrescendoTagField;
        this.intakeCameraFree = true;
        swifferPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_LAST_POSE, 
            swifferCamera, 
            VisionConstants.kRobotToSwifferCam);

        intakePoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_LAST_POSE, 
            intakeCamera, 
            VisionConstants.kRobotToIntakeCam
            );
        swifferPoseEstimator.setLastPose(new Pose2d(15.2,5.5,new Rotation2d(Math.toRadians(180))));
        intakePoseEstimator.setLastPose(new Pose2d(15.2,5.5,new Rotation2d(Math.toRadians(180))));
    } 

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        var result = this.swifferCamera.getLatestResult();
        if (result.hasTargets()) {
            var latestPose = swifferPoseEstimator.update(result);
            if (latestPose.isPresent()) {
                this.latestEstimatedPose = latestPose.get();
            }
            return latestPose;
        }
        return Optional.empty();
    }

    @Override
    public void periodic(){
        var swifferResult = this.swifferCamera.getLatestResult();

        if (swifferResult.hasTargets()) {
            var target = swifferResult.getBestTarget();
            Pose3d bestEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToSwifferCam);
            // Pose3d altEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getAlternateCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToCam);
            
            var estimatedPose = swifferPoseEstimator.update(swifferResult);
        
            if (estimatedPose.isPresent() && this.swifferCamera.getLatestResult().hasTargets() && !DriverStation.isAutonomousEnabled()){
                swerve.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
            }
        }

        // var intakeResult = this.intakeCamera.getLatestResult();
        // if (intakeResult.hasTargets()) {
        //     var target = intakeResult.getBestTarget();
        //     Pose3d bestEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToIntakeCam);
        //     // Pose3d altEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getAlternateCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToCam);
            
        //     var estimatedPose = swifferPoseEstimator.update(intakeResult);
        
        //     if (estimatedPose.isPresent() && this.intakeCamera.getLatestResult().hasTargets() && !DriverStation.isAutonomousEnabled()){
        //         swerve.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
        //     }
        // }
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