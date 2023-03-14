// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import javax.swing.plaf.ToolBarUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Tongue;

public class DefaultMouth extends CommandBase {
  
  private Jaw jaw;
  private Neck neck;
  private Tongue tongue;
  private Grabber grabber;
  private BooleanSupplier lick, grabCone, grabCube;

  public DefaultMouth(BooleanSupplier lick, BooleanSupplier grabCone, BooleanSupplier grabCube, Jaw jaw, Neck neck, Tongue tongue, Grabber grabber) {
    this.jaw = jaw;
    this.neck = neck;
    this.tongue = tongue;
    this.grabber = grabber;

    addRequirements(tongue, grabber);

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
    if(!tongue.isLicked()){
        grabber.grabThang(!grabCone.getAsBoolean());
    }
    if(!tongue.isLicked()){
        grabber.grabThang(!grabCube.getAsBoolean());
    }
  }
  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
