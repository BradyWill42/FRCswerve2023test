// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tongue extends SubsystemBase {

  // private final DoubleSolenoid tongue;
  private final Solenoid thrustTongue, pullOutTongue;
  private boolean isLicked;

  
  /** Creates a new Climber. */
  public Tongue() {
    // tongue = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.Snake.boopID1, Constants.Snake.boopID2); 
    thrustTongue = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.boopID1);
    pullOutTongue = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.boopID2);

    lick(false);
    // tongue.set(DoubleSolenoid.Value.kForward);
  }


  public void lick(boolean lick){
    thrustTongue.set(lick);
    pullOutTongue.set(!lick);

    isLicked = lick;
  }

  public boolean isLicked(){
    return isLicked;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
