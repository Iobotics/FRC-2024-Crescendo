package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.CANSparkMaxUtil;
import frc.robot.Utils.CANSparkMaxUtil.Usage;
import frc.robot.Utils.Conversions;
import frc.robot.Utils.REVModuleState;
import frc.robot.Utils.SwerveModuleConstants;
import frc.robot.Utils.SwerveModuleInterface;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

public class SwerveModule implements SwerveModuleInterface{
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private SparkPIDController mAnglePID;
    private CANcoder angleEncoder;

    private CANSparkMax mDriveMotor;
    private SparkPIDController mDrivePID;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private double abs;
    private boolean invertedA;
    private boolean invertedD;
    private double CANoffsets;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        this.invertedA = moduleConstants.invertedA;
        this.invertedD = moduleConstants.invertedD;
        this.CANoffsets = moduleConstants.CANoffsets;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "Carnivore");
        configCAN();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        mAnglePID = mAngleMotor.getPIDController();
        configAngle();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        mDrivePID = mDriveMotor.getPIDController();
        configDrive();



        lastAngle = getState().angle;
    }

    public void configDrive(){
        mDriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(invertedD);
        mDriveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);

        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
        
        mDrivePID.setP(Constants.SwerveConstants.angleKP);
        mDrivePID.setI(Constants.SwerveConstants.angleKI);
        mDrivePID.setD(Constants.SwerveConstants.angleKD);
        mDrivePID.setFF(Constants.SwerveConstants.angleKF);

        mDriveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);

    }
    
    public void configAngle(){
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(invertedA);
        mAngleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);

        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        
        mAnglePID.setP(Constants.SwerveConstants.angleKP);
        mAnglePID.setI(Constants.SwerveConstants.angleKI);
        mAnglePID.setD(Constants.SwerveConstants.angleKD);
        mAnglePID.setFF(Constants.SwerveConstants.angleKF);

        mAngleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    public void configCAN(){
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cfg.MagnetSensor.MagnetOffset = CANoffsets;
        angleEncoder.getConfigurator().apply(cfg);
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = REVModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToSpark(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDrivePID.setReference(velocity, ControlType.kVelocity, 0, driveFeedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAnglePID.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.sparkToDegrees(integratedAngleEncoder.getPosition(), Constants.SwerveConstants.angleGearRatio));
    }

    public Rotation2d getCANcoder(){
        angleEncoder.getAbsolutePosition().refresh();
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360);
    }

    public void resetToAbsolute(){ 
        double absolutePosition = getCANcoder().getDegrees() - angleOffset.getDegrees();
        abs = absolutePosition;
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    public double getAbsolutePosition(){
        return abs;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveEncoder.getVelocity(), 
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getAngle()
        );
    }

}