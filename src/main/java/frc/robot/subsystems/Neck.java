package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Neck extends SubsystemBase {

  private CANSparkMax leftNeckMotor, rightNeckMotor;
  private Encoder leftNeckEncoder, rightNeckEncoder;
  private SparkMaxPIDController leftNeckPIDController, rightNeckPIDController;
  // private final DoubleSolenoid brake;
  private final Solenoid brakeEnabled, brakeDisabled;

  private final double maxPosition = 1.5;
  private final double minPosition = 0.01;
  private final double shaftDiameter = Units.inchesToMeters(1.0);

  public Neck() {
    leftNeckMotor = new CANSparkMax(Constants.Snake.leftNeckMotorID,  MotorType.kBrushless);
    rightNeckMotor = new CANSparkMax(Constants.Snake.rightNeckMotorID,  MotorType.kBrushless);
    // brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Snake.brakeID1, Constants.Snake.brakeID2);
    brakeEnabled = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.brakeID1);
    brakeDisabled = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.brakeID2);
    
    leftNeckEncoder = new Encoder(Constants.Snake.leftNeckEncoderID1, Constants.Snake.leftNeckEncoderID2);
    rightNeckEncoder = new Encoder(Constants.Snake.rightNeckEncoderID1, Constants.Snake.rightNeckEncoderID2);
    
    initPID();
    resetMotors();

    // brake.set(DoubleSolenoid.Value.kForward);
    brakeEnabled.set(true);
    brakeDisabled.set(false);

    setEncoderCoversions();
  }

  public void initPID(){
    leftNeckPIDController = leftNeckMotor.getPIDController();
    rightNeckPIDController = rightNeckMotor.getPIDController();

    leftNeckPIDController.setP(Constants.Snake.neckPP, Constants.Snake.neckPSlot);
    leftNeckPIDController.setI(Constants.Snake.neckPI, Constants.Snake.neckPSlot);
    leftNeckPIDController.setD(Constants.Snake.neckPD, Constants.Snake.neckPSlot);
    leftNeckPIDController.setFF(Constants.Snake.neckPF, Constants.Snake.neckPSlot);

    rightNeckPIDController.setP(Constants.Snake.neckVP, Constants.Snake.neckVSlot);
    rightNeckPIDController.setI(Constants.Snake.neckVI, Constants.Snake.neckVSlot);
    rightNeckPIDController.setD(Constants.Snake.neckVD, Constants.Snake.neckVSlot);
    rightNeckPIDController.setFF(Constants.Snake.neckVF, Constants.Snake.neckVSlot);
  }

  public void setEncoderCoversions(){
    rightNeckEncoder.setReverseDirection(true);

    leftNeckEncoder.setDistancePerPulse((1.0 / 2048.0) * (Math.PI * shaftDiameter));
    rightNeckEncoder.setDistancePerPulse((1.0 / 2048.0) * (Math.PI * shaftDiameter));
  }

  public void resetMotors(){
    leftNeckMotor.restoreFactoryDefaults();
    rightNeckMotor.restoreFactoryDefaults();

    leftNeckMotor.setIdleMode(Constants.Snake.leftNeckNeutralMode);
    rightNeckMotor.setIdleMode(Constants.Snake.rightNeckNeutralMode);
    
    rightNeckMotor.follow(leftNeckMotor);
  }

  // public void toggleBrake(){
  //   brake.toggle();
  // }

  public void enableBrakes(boolean isBraked){
    brakeDisabled.set(!isBraked);
    brakeEnabled.set(isBraked);
  }
  

  public void resetArmEncoders() {
    leftNeckEncoder.reset();
    rightNeckEncoder.reset();
  }
    
  public double getLeftNeckDistance(){
    return leftNeckEncoder.getDistance();
  }

  public double getRightNeckDistance(){
    return rightNeckEncoder.getDistance();
  }
  
  public void neckOut() {
    if(leftNeckEncoder.getDistance() < maxPosition){
      leftNeckMotor.set(.7);
      rightNeckMotor.set(.7);
    }
  }

  public void neckIn() {
    //if(leftNeckEncoder.getDistance() > minPosition){
      leftNeckMotor.set(-.7);
      rightNeckMotor.set(-.7);
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
    leftNeckPIDController.setReference(position, ControlType.kPosition);
    rightNeckPIDController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Extend Arm Encoder", arm.getSelectedSensorPosition());

    SmartDashboard.putNumber("leftNeckDistance", getLeftNeckDistance());
    SmartDashboard.putNumber("rightNeckDistance", getRightNeckDistance());

  }
}
