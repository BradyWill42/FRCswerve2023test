package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Neck extends SubsystemBase {

  private final CANSparkMax leftNeckMotor, rightNeckMotor, jawMotor;
  private final Encoder leftNeckEncoder, rightNeckEncoder;
  private final SparkMaxAbsoluteEncoder jawEncoder;
  private final double shaftCircumference = Units.inchesToMeters(1.0);

  public Neck() {
    leftNeckMotor = new CANSparkMax(Constants.Snake.leftNeckMotorID,  MotorType.kBrushless);
    rightNeckMotor = new CANSparkMax(Constants.Snake.rightNeckMotorID,  MotorType.kBrushless);
    jawMotor = new CANSparkMax(Constants.Snake.jawMotorID,  MotorType.kBrushless);
    
    leftNeckEncoder = new Encoder(Constants.Snake.leftNeckEncoderID1, Constants.Snake.leftNeckEncoderID2);
    rightNeckEncoder = new Encoder(Constants.Snake.rightNeckEncoderID1, Constants.Snake.rightNeckEncoderID2);
    // jawEncoder = jawMotor.getEncoder();
    jawEncoder = jawMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftNeckMotor.restoreFactoryDefaults();
    rightNeckMotor.restoreFactoryDefaults();
    jawMotor.restoreFactoryDefaults();
    
    leftNeckMotor.setIdleMode(Constants.Snake.leftNeckNeutralMode);
    rightNeckMotor.setIdleMode(Constants.Snake.rightNeckNeutralMode);
    jawMotor.setIdleMode(Constants.Snake.jawNeutralMode);  

    leftNeckEncoder.setDistancePerPulse((1.0 / 8192.0) * (Math.PI * shaftCircumference));
    rightNeckEncoder.setDistancePerPulse((1.0 / 8192.0) * (Math.PI * shaftCircumference) );
    jawEncoder.setPositionConversionFactor((1/Constants.Snake.jawGearRatio) // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
            * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
  
    rightNeckMotor.follow(leftNeckMotor);

  }

  // Check if motor is at its forward soft limit

  public void resetArmEncoders() {
    leftNeckEncoder.reset();
    rightNeckEncoder.reset();
  }
  
  public void resetjawEncoder() {
    jawEncoder.setZeroOffset(jawEncoder.getPosition());
  }
  
  public double getLeftNeckDistance(){
    return leftNeckEncoder.getDistance();
  }

  public double getRightNeckDistance(){
    return rightNeckEncoder.getDistance();
  }
  
  public double getNeckAngle(){
    return jawEncoder.getPosition();
  }
  
 

  public void armOut() {
      leftNeckMotor.set(.3);
      rightNeckMotor.set(.3);
  }

  public void armIn() {
      leftNeckMotor.set(-.3);
      rightNeckMotor.set(-.3);
  }

  public void armOff() {
    leftNeckMotor.set(0);
    rightNeckMotor.set(0);
  }
  
  // public void setArmPosition(double position){
  //   jawMotor.set(CANSparkMax.ControlType.kPosition, position);
  // }
  
  public void armUp(){
    jawMotor.set(.9); 
  }
  
  public void armDown(){
    jawMotor.set(-.9); 
  }

  public void angleArmOff(){
    jawMotor.set(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Extend Arm Encoder", arm.getSelectedSensorPosition());
  }
}
