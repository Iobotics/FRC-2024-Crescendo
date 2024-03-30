package frc.robot.Commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision;

public class AutoIntakeNote extends Command{
    private Vision vision;
    private PhotonCamera camera;
    private Swerve swerve;
    private Intake intake;
    // private Arm arm;
    private PIDController translationController;
    private PIDController strafeController;

    public AutoIntakeNote(Vision vision, Swerve swerve, Intake intake) {
        this.vision = vision;
        this.camera = vision.intakeCamera;
        this.swerve = swerve;
        this.intake = intake;
        // this.arm = arm;

        this.translationController = new PIDController(0.2, 0, 0);
        this.strafeController = new PIDController(0.1, 0, 0);
        
        addRequirements(swerve);
        // -10 pitch 
        // -20 pitch 0 yaw

    }

    @Override
    public void initialize() {
        this.vision.intakeCameraFree = false;
        this.camera.setPipelineIndex(VisionConstants.COLORED_SHAPE_PIPELINE);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("started", true);
        var result = this.camera.getLatestResult();

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            SmartDashboard.putNumber("ringYaw", target.getYaw());
            SmartDashboard.putNumber("ringPitch", target.getPitch());

            if (!intake.optic()) {
                swerve.drive(new Translation2d(0,0), 0,false, true);
                return;
            }

            swerve.drive(
                new Translation2d(
                    -translationController.calculate(target.getPitch(),-20), 
                    strafeController.calculate(target.getYaw(),0)), 
                    0, false, true
                );

            // if (target.getPitch() < -10) {
            //     arm.setArmPos(-9.6);
                
            // }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.camera.setPipelineIndex(VisionConstants.APRILTAG_PIPELINE);
        this.vision.intakeCameraFree = true;
    }

    @Override
    public boolean isFinished() {
        return !intake.optic();
    }
}
