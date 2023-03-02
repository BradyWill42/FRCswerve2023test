// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {


  private final DoubleSolenoid gorillaGripper;
  // private final Solenoid grab, release;

  
  /** Creates a new Climber. */
  public Grabber() {
    gorillaGripper = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.Snake.grabberID1, Constants.Snake.grabberID2); 
    
    // grab = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.grabberID1);
    // release = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.grabberID2);

    gorillaGripper.set(DoubleSolenoid.Value.kForward);

  }

  public void grab(){
    gorillaGripper.toggle();
	}

  // public void grabThang(boolean toGrab){
  //   release.set(!toGrab);
  //   grab.set(toGrab);
  // }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
