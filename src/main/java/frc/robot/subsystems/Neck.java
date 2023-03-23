package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Snake;

public class Neck extends SubsystemBase {

  private CANSparkMax leftNeckMotor, rightNeckMotor;
  private RelativeEncoder leftNeckEncoder, rightNeckEncoder;

  // private RelativeEncoder leftNeckTestEncoder, rightNeckTestEncoder;
  
  private SparkMaxPIDController leftNeckPIDController, rightNeckPIDController;

  // private final DoubleSolenoid brake;
  private final Solenoid brakeEnabled, brakeDisabled;

  private final double maxPosition = 1.5;
  private final double minPosition = 0.01;
  private final double shaftDiameter = Units.inchesToMeters(1.25);

  public Neck() {
    leftNeckMotor = new CANSparkMax(Constants.Snake.leftNeckMotorID,  MotorType.kBrushless);
    rightNeckMotor = new CANSparkMax(Constants.Snake.rightNeckMotorID,  MotorType.kBrushless);
    // brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Snake.brakeID1, Constants.Snake.brakeID2);
    brakeEnabled = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Snake.brakeID1);
    brakeDisabled = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Snake.brakeID2);

    

    // leftNeckTestEncoder = leftNeckMotor.getEncoder();
    // rightNeckTestEncoder = rightNeckMotor.getEncoder();
    
    leftNeckEncoder = leftNeckMotor.getEncoder();
    rightNeckEncoder = rightNeckMotor.getEncoder();
    
    resetMotors();
    setEncoderCoversions();
    initPIDandBurnFlash();
    

    // brake.set(DoubleSolenoid.Value.kForward);
    brakeEnabled.set(false);
    brakeDisabled.set(true);

  }

  public void initPIDandBurnFlash(){
    leftNeckPIDController = leftNeckMotor.getPIDController();
    rightNeckPIDController = rightNeckMotor.getPIDController();

    leftNeckPIDController.setP(Constants.Snake.neckPP, Constants.Snake.neckPSlot);
    leftNeckPIDController.setI(Constants.Snake.neckPI, Constants.Snake.neckPSlot);
    leftNeckPIDController.setD(Constants.Snake.neckPD, Constants.Snake.neckPSlot);
    leftNeckPIDController.setFF(Constants.Snake.neckPF, Constants.Snake.neckPSlot);

    rightNeckPIDController.setP(Constants.Snake.neckPP, Constants.Snake.neckPSlot);
    rightNeckPIDController.setI(Constants.Snake.neckPI, Constants.Snake.neckPSlot);
    rightNeckPIDController.setD(Constants.Snake.neckPD, Constants.Snake.neckPSlot);
    rightNeckPIDController.setFF(Constants.Snake.neckPF, Constants.Snake.neckPSlot);

    
    rightNeckMotor.burnFlash();
    leftNeckMotor.burnFlash();

  }

  public void setEncoderCoversions(){
    
    rightNeckEncoder.setPositionConversionFactor((Math.PI * shaftDiameter) * (1.0 / Constants.Snake.neckGearRatio));
    leftNeckEncoder.setPositionConversionFactor((Math.PI * shaftDiameter) * (1.0 / Constants.Snake.neckGearRatio));
    
    // leftNeckEncoder.setDistancePerPulse((1.0 / 2048.0) * (Math.PI * shaftDiameter));
    // rightNeckEncoder.setDistancePerPulse((1.0 / 2048.0) * (Math.PI * shaftDiameter));
  }

  public void resetMotors(){
    leftNeckMotor.restoreFactoryDefaults();
    rightNeckMotor.restoreFactoryDefaults();

    leftNeckMotor.setIdleMode(Constants.Snake.leftNeckNeutralMode);
    rightNeckMotor.setIdleMode(Constants.Snake.rightNeckNeutralMode);
   
    // rightNeckMotor.follow(leftNeckMotor);
  }


  public void enableBrakes(boolean isBraked){
    brakeDisabled.set(!isBraked);
    brakeEnabled.set(isBraked);
  }
  

  public void resetArmEncoders() {
    leftNeckEncoder.setPosition(0);
    rightNeckEncoder.setPosition(0);
  }
    
  public double getLeftNeckDistance(){
    return leftNeckEncoder.getPosition();
  }

  public double getRightNeckDistance(){
    return rightNeckEncoder.getPosition();
  }

  public double getNeckDistance(){
    return ((getLeftNeckDistance() + getRightNeckDistance()) / 2.0);
  }
  
  public void neckOut() {
      enableBrakes(false);
      leftNeckMotor.set(.9);
      rightNeckMotor.set(.9);
  }

  public void neckIn() {
    //if(leftNeckEncoder.getDistance() > minPosition){
      enableBrakes(false);

      leftNeckMotor.set(-.9);
      rightNeckMotor.set(-.9);
    //}
  }
  public void rightNeckIn(){
    rightNeckMotor.set(-.7);
  }
  public void rightNeckOut(){
    leftNeckMotor.set(.7);
  }
  public void leftNeckIn(){
    leftNeckMotor.set(-.7);
  }
  public void leftNeckOut(){
    rightNeckMotor.set(.7);
  }

  public void neckOff() {
    leftNeckMotor.set(0);
    rightNeckMotor.set(0);

    enableBrakes(true);
  }
  
  public void setNeckPosition(double position){
    leftNeckPIDController.setReference(position, ControlType.kPosition, 0);
    rightNeckPIDController.setReference(position, ControlType.kPosition, 0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Extend Arm Encoder", arm.getSelectedSensorPosition());

    SmartDashboard.putNumber("leftNeckDistance", getLeftNeckDistance());
    SmartDashboard.putNumber("rightNeckDistance", getRightNeckDistance());

    SmartDashboard.putNumber("Raw Neck Right Encoder Values", leftNeckEncoder.getPosition());

  }
}
