package frc.robot.Subsystems;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision extends SubsystemBase{
    public final PhotonCamera swifferCamera;
    public final PhotonCamera intakeCamera;
    public double latestAngle;

    private AprilTagFieldLayout aprilTagFieldLayout;
    public final PhotonPoseEstimator swifferEstimator;
    public final PhotonPoseEstimator intakeEstimator;
    // private final Field2d m_field = new Field2d();
    private Swerve swerve;
    private EstimatedRobotPose latestEstimatedPose;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        this.swifferCamera = new PhotonCamera(VisionConstants.kFrontCameraName);
        this.intakeCamera = new PhotonCamera(VisionConstants.kIntakeCameraName);
        aprilTagFieldLayout = VisionConstants.k2024CrescendoTagField;
        swifferEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_LAST_POSE, 
            swifferCamera, 
            VisionConstants.kRobotToSwifferCam);

        this.swifferEstimator.setLastPose(swerve.getEstPose());
        intakeEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_LAST_POSE, 
            intakeCamera, 
            VisionConstants.kRobotToIntakeCam);

        this.swifferEstimator.setLastPose(swerve.getEstPose());
    } 

    // public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    //     var swifferResult = this.swifferCamera.getLatestResult();
    //     if (swifferResult.hasTargets()) {
    //         var latestPose = photonPoseEstimator.update(swifferResult);
    //         if (latestPose.isPresent()) {
    //             this.latestEstimatedPose = latestPose.get();
    //         }
    //         return latestPose;
    //     }
    //     return Optional.empty();
    // }

    // public double getEstimatedYaw() {
    //     if (this.latestEstimatedPose == null) {
    //         return 0;
    //     }
    //     double estimatedYaw = -Math.toDegrees(this.latestEstimatedPose.estimatedPose.getRotation().getZ());
    //     SmartDashboard.putNumber("estimatedYaw", estimatedYaw);
    //     return estimatedYaw;
    // }

    // public double getAngleToGoal(double goalX, double goalY) {
    //     Pose2d goalPose = new Pose2d(goalX, goalY, new Rotation2d());
    //     Pose2d currentPose = swerve.getEstPose();
    //     double goalRotation = Math.toDegrees(Math.atan((currentPose.getY()-goalPose.getY())/(currentPose.getX()-goalPose.getX())));
    //     SmartDashboard.putNumber("toSpeaker", goalRotation);
    //     SmartDashboard.putNumber("goalX", currentPose.minus(goalPose).getX());
    //     SmartDashboard.putNumber("goalY", currentPose.minus(goalPose).getY());
    //     return goalRotation;
    // }

    // public double getEstimatedYaw() {
    //     var estimatedPose = getEstimatedRobotPose();
    //     if (estimatedPose.isPresent()) {
    //         SmartDashboard.putNumber("estimatedrots", estimatedPose.get().estimatedPose.getRotation().getZ());
    //         return estimatedPose.get().estimatedPose.getRotation().getAngle();
    //     }
    //     return 0;
    // }

    public void updatePoseEstimators(PhotonCamera camera) {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            Pose3d bestIntakeEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToIntakeCam);
            Pose3d altIntakeEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getAlternateCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToIntakeCam);

            swerve.getEstPose().nearest(Arrays.asList(bestIntakeEstimate.toPose2d(),altIntakeEstimate.toPose2d()));

            var estimatedPose = intakeEstimator.update(result);
        
            if (estimatedPose.isPresent() && camera.getLatestResult().hasTargets()){
                swerve.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
            }
        }
    }

    @Override
    public void periodic(){
        var swifferResult = this.swifferCamera.getLatestResult();
        if (swifferResult.hasTargets()) {
            var target = swifferResult.getBestTarget();
            Pose3d bestSwifferEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToSwifferCam);
            Pose3d altSwifferEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getAlternateCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToSwifferCam);
            
            this.latestAngle = bestSwifferEstimate.toPose2d().getRotation().getDegrees();
            var estimatedPose = swifferEstimator.update(swifferResult);
        
            if (estimatedPose.isPresent() && this.swifferCamera.getLatestResult().hasTargets()){
                swerve.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
            }
        }  

        if (this.intakeCamera.getPipelineIndex() == 1) {
            var intakeResult = this.intakeCamera.getLatestResult();
            if (intakeResult.hasTargets()) {
                var target = intakeResult.getBestTarget();
                Pose3d bestIntakeEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToIntakeCam);
                Pose3d altIntakeEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getAlternateCameraToTarget(),this.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToIntakeCam);

                swerve.getEstPose().nearest(Arrays.asList(bestIntakeEstimate.toPose2d(),altIntakeEstimate.toPose2d()));

                var estimatedPose = intakeEstimator.update(intakeResult);
            
                if (estimatedPose.isPresent() && this.intakeCamera.getLatestResult().hasTargets()){
                    swerve.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
                }
            }
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