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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision extends SubsystemBase{
    public final PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    public final PhotonPoseEstimator photonPoseEstimator;
    private final Field2d m_field = new Field2d();
    private Swerve swerve;


    public Vision(Swerve swerve) {
        this.swerve = swerve;
        camera = new PhotonCamera(VisionConstants.kFrontCameraName);
        aprilTagFieldLayout = VisionConstants.k2024CrescendoTagField;
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.LOWEST_AMBIGUITY, 
            camera, 
            VisionConstants.kRobotToCam);

        // try {
        //     aprilTagFieldLayout = new AprilTagFieldLayout("2024-crescendo.json");
        // } catch (IOException e) {
        //     e.printStackTrace();
        //     aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        // }
    } 

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            return photonPoseEstimator.update(result);
        }
        return Optional.empty();
    }

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

    @Override
    public void periodic(){
        var estimatedPose = getEstimatedRobotPose();
        if (estimatedPose.isPresent() && this.camera.getLatestResult().hasTargets()){
            swerve.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
        }
        m_field.setRobotPose(swerve.poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field", m_field);
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