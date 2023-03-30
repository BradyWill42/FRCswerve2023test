// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.plaf.ToolBarUI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Tongue;

public class DefaultTongue extends CommandBase {
  
  private Tongue tongue;
  private Grabber grabber;
  private Jaw jaw;
  private BooleanSupplier lick, grabCone, grabCube;
  private boolean psi_60, psi_30;

  public DefaultTongue(BooleanSupplier lick, BooleanSupplier grabCone, BooleanSupplier grabCube, Tongue tongue, Grabber grabber, Jaw jaw) {
    this.tongue = tongue;
    this.grabber = grabber;
    this.jaw = jaw;
    // this.changePressure = changePressure;


    // this.leftAxis = leftAxis;
    // this.rightAxis = rightAxis;

    addRequirements(tongue);//, grabber);

    this.lick = lick;
    this.grabCone = grabCone;
    this.grabCube = grabCube;
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {

    if(jaw.getJawAngle() > 30){
      tongue.lick(lick.getAsBoolean());
    }
    
  }


  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
