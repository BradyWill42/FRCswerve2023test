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

public class DefaultGrabber extends CommandBase {
  
  private Tongue tongue;
  private Grabber grabber;
  private BooleanSupplier lick, grabCone, grabCube;
  private boolean psi_60, psi_30;

  public DefaultGrabber(BooleanSupplier lick, BooleanSupplier grabCone, BooleanSupplier grabCube, Tongue tongue, Grabber grabber) {
    this.tongue = tongue;
    this.grabber = grabber;
    // this.changePressure = changePressure;


    // this.leftAxis = leftAxis;
    // this.rightAxis = rightAxis;

    addRequirements(grabber);//, grabber);

    this.lick = lick;
    this.grabCone = grabCone;
    this.grabCube = grabCube;
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {

    SmartDashboard.putBoolean("Grab", grabCone.getAsBoolean());

    SmartDashboard.putBoolean("lick", lick.getAsBoolean());
    
    // if(grabCone.getAsBoolean()){
    //   psi_60 = true;
    // }
    
    // if(grabCube.getAsBoolean()){
    //   psi_60 = false;
    // }
    
    // tongue.lick(lick.getAsBoolean());
    
    if(!tongue.isLicked()){
      grabber.grabThang(grabCone.getAsBoolean());
    }

    grabber.switchPressure(grabCube.getAsBoolean());
  }


  @Override
  public void end(boolean interrupted) {

  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
