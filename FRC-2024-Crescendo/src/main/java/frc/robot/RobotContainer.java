package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
// import frc.robot.Commands.PresetArm;
// import frc.robot.Commands.ApriltagAlign;
import frc.robot.Commands.PresetWrist;
// import frc.robot.Commands.TeleopSwerve;
// import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Swiffer;



public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick gamepad = new Joystick(OIConstants.kGamepad);
 
    // /* Drive Controls */
    // private final int translationAxis = joystick1.getYChannel();
    // private final int strafeAxis = joystick1.getXChannel();
    // private final int rotationAxis = joystick2.getXChannel();
    // private double scalar = 1.5;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(logi, 5);
    // private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    // private final JoystickButton consume = new JoystickButton(joystick1, 2);
    // private final JoystickButton eject = new JoystickButton(joystick1, 3);


    /* Subsystems */
    private final Swiffer swiffer = new Swiffer();


    //Allows for Autos to be chosen in Shuffleboard
    // SendableChooser<Command> AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // --- SWIFFER BUTTON CONTROLS --- //

        // === SWIFFER === //

        new JoystickButton(gamepad, 1).whileTrue(
            new RunCommand(()->swiffer.setPowerArm(-gamepad.getY()), swiffer));

        new JoystickButton(gamepad, 1).whileFalse(
            new RunCommand(()->swiffer.stopArm(), swiffer));

        new JoystickButton(gamepad, 2).whileTrue(
            new RunCommand(()->swiffer.setPowerWrist(gamepad.getY()), swiffer));

        new JoystickButton(gamepad, 2).whileFalse(
            new RunCommand(()->swiffer.stopWrist(), swiffer)); 

        // new JoystickButton(gamepad, 3).onTrue(
        //     new PresetArm(swiffer, 15));
        
        // new JoystickButton(gamepad, 4).onTrue(
        //     new PresetArm(swiffer, 50));

        new JoystickButton(gamepad, 5).whileTrue(
            new RunCommand(()->swiffer.setPowerRoller(0.75), swiffer));

        new JoystickButton(gamepad, 5).whileFalse(
            new RunCommand(()->swiffer.stopRoller(), swiffer));

        new JoystickButton(gamepad, 6).whileTrue(
            new RunCommand(()->swiffer.setPowerRoller(gamepad.getY()), swiffer));

        new JoystickButton(gamepad, 6).whileFalse(
            new RunCommand(()->swiffer.stopRoller(), swiffer));

        
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }









}
