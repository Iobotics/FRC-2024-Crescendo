package frc.robot.Commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Vision;
import frc.robot.Constants;


public class ApriltagAlign extends Command {
    protected Vision vision;
    private double goal;
    private PIDController translationController;
    private PIDController turnController;
    private PIDController strafeController;


    public ApriltagAlign(Vision vision, double goal) {
        this.vision = vision;
        this.goal = goal;

        final double TRANSLATION_P = 0.1;
        final double TRANSLATION_D = 0.0;
        this.translationController = new PIDController(TRANSLATION_P, 0, TRANSLATION_D);

        final double ANGULAR_P = 0.1;
        final double ANGULAR_D = 0.0;
        this.turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

        final double STRAFE_P = 0.1;
        final double STRAFE_D = 0.0;
        this.strafeController = new PIDController(STRAFE_P, 0, STRAFE_D);


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

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            translationSpeed = -translationController.calculate(bestCamToTarget.getX(), goal);

            strafeSpeed = -strafeController.calculate(bestCamToTarget.getY(),0);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(target.getYaw(), 0);


        } else {
            // If we have no targets, stay still.
            translationSpeed = 0;
            rotationSpeed = 0;
            strafeSpeed = 0;
        }
        // drive(translationSpeed, strafeSpeed, rotationSpeed)
    }
}