package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public final class CTREConfigs {

    public CANSparkMax swerveAngleMotor;
    public CANSparkMax swerveDriveMotor;

    public SparkMaxPIDController swerveAnglePidController;
    public SparkMaxPIDController swerveDrivePidController; 

    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        // swerveAngleMotor = angleSpark;
        // swerveDriveMotor = driveSpark;

        // swerveAnglePidController = swerveAngleMotor.getPIDController();
        // swerveDrivePidController = swerveDriveMotor.getPIDController();

        // swerveAngleFXConfig = new TalonFXConfiguration();
        // swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        // /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.angleEnableCurrentLimit, 
        //     Constants.Swerve.angleContinuousCurrentLimit, 
        //     Constants.Swerve.anglePeakCurrentLimit, 
        //     Constants.Swerve.anglePeakCurrentDuration);

        // swerveAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit, Constants.Swerve.anglePeakCurrentLimit);    

        // swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        // swerveAnglePidController.setP(Constants.Swerve.angleKP, 0);
        // swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        // swerveAnglePidController.setI(Constants.Swerve.angleKI, 0);
        // swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        // swerveAnglePidController.setD(Constants.Swerve.angleKD, 0);
        // swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        // swerveAnglePidController.setFF(Constants.Swerve.angleKF, 0);
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        // /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.driveEnableCurrentLimit, 
        //     Constants.Swerve.driveContinuousCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentDuration);

        // swerveDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit, Constants.Swerve.drivePeakCurrentLimit);

        // swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        // swerveDrivePidController.setP(Constants.Swerve.driveKP, 0);
        // swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        // swerveDrivePidController.setI(Constants.Swerve.driveKI, 0);
        // swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        // swerveDrivePidController.setD(Constants.Swerve.driveKD, 0);
        // swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF; 
        // swerveDrivePidController.setFF(Constants.Swerve.driveKF, 0);       
        // swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        // swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        // swerveDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        // swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        // swerveDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);

        // /* Burn to Can Devices */

        // swerveAngleMotor.burnFlash();
        // swerveDriveMotor.burnFlash();
            
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    public void configAngle(CANSparkMax angleMotor){
        swerveAngleMotor = angleMotor;
        swerveAnglePidController = swerveAngleMotor.getPIDController();

        /* Swerve Angle Motor Configurations */
        swerveAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit, Constants.Swerve.anglePeakCurrentLimit);    

        swerveAnglePidController.setP(Constants.Swerve.angleKP, 0);
        swerveAnglePidController.setI(Constants.Swerve.angleKI, 0);
        swerveAnglePidController.setD(Constants.Swerve.angleKD, 0);
        swerveAnglePidController.setFF(Constants.Swerve.angleKF, 0);

        swerveAngleMotor.burnFlash();
    }
    public void configDrive(CANSparkMax driveMotor){
        swerveDriveMotor = driveMotor;
        swerveDrivePidController = swerveDriveMotor.getPIDController();

        /* Swerve Angle Motor Configurations */
        swerveDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit, Constants.Swerve.drivePeakCurrentLimit);    

        swerveDrivePidController.setP(Constants.Swerve.driveKP, 0);
        swerveDrivePidController.setI(Constants.Swerve.driveKI, 0);
        swerveDrivePidController.setD(Constants.Swerve.driveKD, 0);
        swerveDrivePidController.setFF(Constants.Swerve.driveKF, 0);
        
        swerveDriveMotor.burnFlash();
    }

    public CANCoderConfiguration getCANConfig(){

        CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;


        return swerveCanCoderConfig;
    }
}