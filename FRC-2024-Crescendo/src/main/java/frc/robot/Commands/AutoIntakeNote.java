package frc.robot.Commands;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;

public class AutoIntakeNote extends Command{
    private PhotonCamera camera;
    private Swerve swerve;
    private Intake intake;
    // private Arm arm;
    private PIDController translationController;
    private PIDController strafeController;

    public AutoIntakeNote(PhotonCamera camera, Swerve swerve, Intake intake) {
        this.camera = camera;
        this.swerve = swerve;
        this.intake = intake;
        // this.arm = arm;

        this.translationController = new PIDController(0.4, 0, 0);
        this.strafeController = new PIDController(0.4, 0, 0);
        
        addRequirements(swerve);
        // -10 pitch 
        // -20 pitch 0 yaw

    }

    @Override
    public void initialize() {
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
    }

    @Override
    public boolean isFinished() {
        return !intake.optic();
    }
}
