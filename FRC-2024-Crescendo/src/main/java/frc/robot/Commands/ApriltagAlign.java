package frc.robot.Commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Vision;
import frc.robot.Constants.VisionConstants;


public class ApriltagAlign extends Command {
    protected Vision vision;
    private double goal;
    private PIDController translationController;
    private PIDController rotationController;
    private PIDController strafeController;

    public ApriltagAlign(Vision vision, double goal) {
        this.vision = vision;
        this.goal = goal;

        this.translationController = new PIDController(VisionConstants.translationKP, VisionConstants.translationKI, VisionConstants.translationKD);

        this.strafeController = new PIDController(VisionConstants.strafeKP, VisionConstants.strafeKI, VisionConstants.strafeKD);

        this.rotationController = new PIDController(VisionConstants.rotationKP,VisionConstants.rotationKI,VisionConstants.rotationKD);
    }


    @Override
    public void execute() {
        double translationSpeed = 0;
        double strafeSpeed = 0;
        double rotationSpeed = 0;
        var result = vision.camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d bestCamToTarget = target.getBestCameraToTarget();

            translationSpeed = -translationController.calculate(bestCamToTarget.getX(), goal);

            strafeSpeed = -strafeController.calculate(bestCamToTarget.getY(),0);

            rotationSpeed = -rotationController.calculate(target.getYaw(), 0);


        } else {
            // If we have no targets, stay still.
            translationSpeed = 0;
            rotationSpeed = 0;
            strafeSpeed = 0;
        }
        // drive(translationSpeed, strafeSpeed, rotationSpeed)
    }
}