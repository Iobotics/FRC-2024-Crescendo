package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;

public class gotoGoal extends Command {
    private Pose2d goalPose;
    private Pose2d estPose;
    private Swerve swerve;
    private PIDController translationController;
    private PIDController strafeController;
    private PIDController rotationController;

    public gotoGoal(Pose2d goalPose, Swerve swerve) {
        this.goalPose = goalPose;
        this.swerve = swerve;
        translationController = new PIDController(0.4, 0.02, 0);
        strafeController = new PIDController(0.4, 0.02, 0);
        rotationController = new PIDController(0.4, 0.04, 0);
    }

    @Override 
    public void initialize() {

    }

    @Override
    public void execute() {
        estPose = swerve.getEstPose();
        swerve.drive(
            new Translation2d(
                -translationController.calculate(estPose.getX(),goalPose.getX()),
                -strafeController.calculate(estPose.getY(),goalPose.getY())
            ),
            rotationController.calculate(estPose.getRotation().getRadians(),goalPose.getRotation().getRadians()), 
            true, 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true, true);
    }

    @Override
    public boolean isFinished() {
        return (estPose.minus(goalPose).getX() < 0.1 && estPose.minus(goalPose).getY() < 0.1 && estPose.minus(goalPose).getRotation().getRadians() < 0.1);
    }
}
