package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utils.Conversions;
import frc.robot.Utils.REVConfigs;
import frc.robot.Utils.SwerveModuleConstants;
import frc.robot.Utils.SwerveModuleInterface;
import frc.robot.Constants;

public class SwerveModule implements SwerveModuleInterface{
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkMax mAngleMotor;
    private SparkPIDController mAnglePID;
    private CANcoder angleEncoder;

    private CANSparkMax mDriveMotor;
    private SparkPIDController mDrivePID;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        REVConfigs.CancoderConfig.applyCancoderConfig(angleEncoder);

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        REVConfigs.AngleConfig.applyAngleConfig(mAngleMotor);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        REVConfigs.DriveConfig.applyDriveConfig(mDriveMotor);
        mDrivePID.setReference(0, ControlType.kVelocity);
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        double angle = desiredState.angle.getRotations();
        mAnglePID.setReference(angle, ControlType.kPosition);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDrivePID.setReference(output, ControlType.kDutyCycle);
        }
        else {
            double velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
            mDrivePID.setReference(velocity, ControlType.kVelocity);

            // TODO not sure if needed or what the comparable REV call is.
            double feedforward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDrivePID.setFF(feedforward);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAnglePID.setReference(absolutePosition, ControlType.kPosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getEncoder().getPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getEncoder().getPosition(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getEncoder().getPosition())
        );
    }
}