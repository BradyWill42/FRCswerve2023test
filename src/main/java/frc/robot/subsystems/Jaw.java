package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AlternateEncoderType;
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

import edu.wpi.first.math.controller.PIDController;
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
  // private RelativeEncoder jawEncoder;
  private Encoder jawEncoder;
  private PIDController jawPIDController;
  private double currentPosition;
  private DigitalInput limitSwitch;

  public Jaw() {
    jawMotor = new CANSparkMax(Constants.Snake.jawMotorID, MotorType.kBrushless);
    jawMotor.setIdleMode(Constants.Snake.jawNeutralMode); 

    limitSwitch = new DigitalInput(Constants.Snake.limitSwitchID);
    // jawEncoder = jawMotor.getEncoder();

    jawEncoder = new Encoder(Constants.Snake.jawEncoderID1, Constants.Snake.jawEncoderID2);
    resetMotors();
    setEncoderCoversions();
    initPID();
    
  }

  public void initPID(){
    jawPIDController = new PIDController(
      Constants.Snake.jawPP,
      Constants.Snake.jawPI,
      Constants.Snake.jawPD
    );

    jawPIDController.setTolerance(2);

    // jawPIDController.setP(Constants.Snake.jawPP, Constants.Snake.jawPSlot);
    // jawPIDController.setI(Constants.Snake.jawPI, Constants.Snake.jawPSlot);
    // jawPIDController.setD(Constants.Snake.jawPD, Constants.Snake.jawPSlot);
    // jawPIDController.setFF(Constants.Snake.jawPF, Constants.Snake.jawPSlot);

    // jawPIDController.setP(Constants.Snake.jawVP, Constants.Snake.jawVSlot);
    // jawPIDController.setI(Constants.Snake.jawVI, Constants.Snake.jawVSlot);
    // jawPIDController.setD(Constants.Snake.jawVD, Constants.Snake.jawVSlot);
    // jawPIDController.setFF(Constants.Snake.jawVF, Constants.Snake.jawVSlot);
  }

  public boolean isPressed(){
    if(limitSwitch.get()){
      jawEncoder.reset();
    }
    return limitSwitch.get();
  }

  public void setEncoderCoversions(){
    // jawEncoder.setPositionConversionFactor((1.0 / Constants.Snake.jawGearRatio) * 360.0); // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
    // jawEncoder.setVelocityConversionFactor(((1.0 / Constants.Snake.jawGearRatio) * 360.0) / 60.0);
    jawEncoder.setDistancePerPulse((1.0 / 2048.0) * 360.0); // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
    jawEncoder.setReverseDirection(true);
    // jawEncoder.set(((1.0 / 2048.0) * 360.0) / 60.0);
  }

  public void resetMotors(){
    jawMotor.restoreFactoryDefaults();
    jawMotor.setIdleMode(Constants.Snake.jawNeutralMode); 
  }

  public void setJawAngle(double angle){
    jawPIDController.setSetpoint(angle);
    jawMotor.set(jawPIDController.calculate(jawEncoder.getDistance(), angle));
  }

  
  
  public void resetjawEncoder() {
    jawEncoder.reset();
  }

  public double getJawAngle(){
    return jawEncoder.getDistance();
  }
  
  public void jawOpen(){
    jawMotor.set(0.4);
  }
  
  public void jawClose(){
    if(!isPressed()){
      jawMotor.set(-0.4);
    } else {
      jawOff();
    }
  }

  public void jawOff(){
    jawMotor.set(0);
  }

  @Override
  public void periodic() {
    isPressed();
    SmartDashboard.putNumber("Neck Angle", getJawAngle());
  }
}
