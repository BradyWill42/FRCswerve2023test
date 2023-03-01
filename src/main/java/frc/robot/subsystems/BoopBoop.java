// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoopBoop extends SubsystemBase {


  private final DoubleSolenoid booper;
  private final Solenoid thrust, pullOut;

  
  /** Creates a new Climber. */
  public BoopBoop() {
    booper = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.Snake.boopID1, Constants.Snake.boopID2); 
    thrust = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.boopID1);
    pullOut = new Solenoid(PneumaticsModuleType.REVPH, Constants.Snake.boopID2);

    booper.set(DoubleSolenoid.Value.kForward);
  }

  public void boop(){
    booper.toggle();
	}

  public void pump(boolean dump){
    thrust.set(dump);
    pullOut.set(!dump);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
