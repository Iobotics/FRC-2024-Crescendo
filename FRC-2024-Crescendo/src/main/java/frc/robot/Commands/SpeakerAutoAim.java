package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;

public class SpeakerAutoAim extends Command{
    Supplier<Pose2d> poseToSpeaker;
    public boolean overrideTeleop;
    // PresetArm setArm;
    Intake intake;
    Swerve swerve;
    TeleopSwerve teleopSwerve;
    ChassisSpeeds prevChassisSpeeds;

    public SpeakerAutoAim(Supplier<Pose2d> poseToSpeaker, Intake intake, Swerve swerve, TeleopSwerve teleopSwerve) {
        this.poseToSpeaker = poseToSpeaker; // call poseToSpeaker.get() for value
        this.intake = intake;
        // this.setArm = setArm;
        this.swerve = swerve;
        this.prevChassisSpeeds = new ChassisSpeeds();
        this.overrideTeleop = false;
        this.teleopSwerve = teleopSwerve;
        // addRequirements(intake);
        this.teleopSwerve.getIfOverriding = this::getIfOverride;
        this.teleopSwerve.rotationOverride = this::getRotation;
        
    }

    public boolean getIfOverride() {
        return overrideTeleop;
    }

    public double getRotation() {
        return MathUtil.clamp(swerve.rotateToSpeaker(),-1.0,1.0);
    }

    public void run() {
        SmartDashboard.putBoolean("command_recieved", true);
        this.overrideTeleop = true;
    }

    public void stop(boolean interrupted) {
        SmartDashboard.putBoolean("command_recieved", false);
        this.overrideTeleop = false;
    }

    // @Override
    // public boolean isFinished() {
    //     return !this.overrideTeleop;
    // }

    
}
