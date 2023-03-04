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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Jaw extends SubsystemBase {

  private CANSparkMax jawMotor;
  private RelativeEncoder jawEncoder;
  private SparkMaxPIDController jawPIDController;
  private double currentPosition;

  public Jaw() {
    jawMotor = new CANSparkMax(Constants.Snake.jawMotorID,  MotorType.kBrushless);
    jawMotor.setIdleMode(Constants.Snake.jawNeutralMode); 

    jawEncoder = jawMotor.getEncoder();
    initPID();


    setEncoderCoversions();
  }

  public void initPID(){
    jawPIDController = jawMotor.getPIDController();


    jawPIDController.setP(Constants.Snake.jawPP, Constants.Snake.jawPSlot);
    jawPIDController.setI(Constants.Snake.jawPI, Constants.Snake.jawPSlot);
    jawPIDController.setD(Constants.Snake.jawPD, Constants.Snake.jawPSlot);
    jawPIDController.setFF(Constants.Snake.jawPF, Constants.Snake.jawPSlot);

    jawPIDController.setP(Constants.Snake.jawVP, Constants.Snake.jawVSlot);
    jawPIDController.setI(Constants.Snake.jawVI, Constants.Snake.jawVSlot);
    jawPIDController.setD(Constants.Snake.jawVD, Constants.Snake.jawVSlot);
    jawPIDController.setFF(Constants.Snake.jawVF, Constants.Snake.jawVSlot);
  }

  public void setEncoderCoversions(){
    jawEncoder.setPositionConversionFactor((1.0 / Constants.Snake.jawGearRatio) * 360.0); // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
    jawEncoder.setVelocityConversionFactor(((1.0 / Constants.Snake.jawGearRatio) * 360.0) / 60.0);
  }

  public void resetMotors(){
    jawMotor.restoreFactoryDefaults();
    jawMotor.setIdleMode(Constants.Snake.jawNeutralMode); 
  }

  public void setJawAngle(double angle){
    jawPIDController.setReference(angle, ControlType.kPosition);
  }
  
  public void resetjawEncoder() {
    jawEncoder.setPosition(0);
  }

  public double getJawAngle(){
    return jawEncoder.getPosition();
  }
  
  public void jawOpen(){
    jawMotor.set(0.4);
    // jawPIDController.setReference(0.4, ControlType.kVelocity);
  }
  
  public void jawClose(){
    jawMotor.set(-0.4);
    // jawPIDController.setReference(-0.4, ControlType.kVelocity);
  }

  public void jawOff(){
    // currentPosition = jawEncoder.getPosition();
    // jawPIDController.setReference(currentPosition, ControlType.kPosition);
    jawPIDController.setReference(0, ControlType.kSmartVelocity);
    jawMotor.set(0);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Neck Angle", getJawAngle());

  }
}
