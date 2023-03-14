// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {


  private final DoubleSolenoid gorillaGripper;
  // private final Solenoid grab, release;

  
  /** Creates a new Climber. */
  public Grabber() {
    gorillaGripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, Constants.Snake.grabberID1, Constants.Snake.grabberID2); 
    
    // grab = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Snake.grabberID1);
    // release = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Snake.grabberID2);

    // grabThang(false);
    gorillaGripper.set(Value.kForward);
  }

  public void grabThang(boolean toGrab){
    if(toGrab){
      gorillaGripper.set(Value.kForward);
    } else if(!toGrab){
      gorillaGripper.set(Value.kReverse);
    } else {
      gorillaGripper.set(Value.kOff);
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
