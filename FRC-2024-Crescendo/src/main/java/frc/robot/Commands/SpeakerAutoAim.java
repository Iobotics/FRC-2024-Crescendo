package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class SpeakerAutoAim extends Command{
    Pose2d poseToSpeaker;
    PresetArm setArm;
    Intake intake;

    public SpeakerAutoAim(Pose2d poseToSpeaker, Intake intake, PresetArm setArm) {
        this.poseToSpeaker = poseToSpeaker;
        this.intake = intake;
        this.setArm = setArm;
        addRequirements(intake);
        
    }



    public void execute() {

    }
}
