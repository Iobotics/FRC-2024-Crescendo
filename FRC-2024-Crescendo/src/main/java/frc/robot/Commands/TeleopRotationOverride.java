package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve;

public class TeleopRotationOverride {
    public boolean overrideTeleop;
    private Supplier<Double> targetRotation;
    private Swerve swerve;
    private TeleopSwerve teleopSwerve;
    private PIDController rotationController;

    /**
     * Class to override teleop rotation while keeping translation in control
     * @param targetRotaion Double Supplier that returns target rotation
     * @param Swerve Swerve subsystem object
     * @param teleopSwerve telop command object
     */
    public TeleopRotationOverride(Supplier<Double> targetRotation, Swerve swerve, TeleopSwerve teleopSwerve) {
        this.targetRotation = targetRotation; // call targetRotation.get() for value
        this.swerve = swerve;
        this.overrideTeleop = false;
        this.teleopSwerve = teleopSwerve;
        // addRequirements(intake);

        this.teleopSwerve.getIfOverriding = this::getIfOverride;
        this.teleopSwerve.rotationOverride = this::getRotation;
        this.rotationController = new PIDController(2.0, 0.0, 0.0);
        this.rotationController.enableContinuousInput(-180, 180);
    }

    public boolean getIfOverride() {
        return overrideTeleop;
    }

    public double getRotation() {
        return MathUtil.clamp(Math.toRadians(rotationController.calculate(swerve.getEstYaw(),targetRotation.get())),-1.0,1.0);
    }

    public void run() {
        SmartDashboard.putBoolean("command_recieved", true);
        this.overrideTeleop = true;
    }

    public void stop(boolean interrupted) {
        SmartDashboard.putBoolean("command_recieved", false);
        this.overrideTeleop = false;
    }
}
