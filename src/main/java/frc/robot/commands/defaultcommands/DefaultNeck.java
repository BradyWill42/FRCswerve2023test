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

public class DefaultNeck extends CommandBase {
  
  private Jaw jaw;
  private Neck neck;
  private Tongue tongue;
  private Grabber grabber;

  private double currentAngle;
  private DoubleSupplier neckIn, neckOut;

  public DefaultNeck(DoubleSupplier neckIn, DoubleSupplier neckOut, Jaw jaw, Neck neck, Tongue tongue, Grabber grabber) {
    this.neck = neck;
    this.tongue = tongue;
    this.grabber = grabber;
    this.neckIn = neckIn;
    this.neckOut = neckOut;

    addRequirements(neck);

  }

  
  @Override
  public void initialize() {
    // currentAngle = jaw.getJawAngle();
  }

  
  @Override
  public void execute() {
    if(neckIn.getAsDouble() > 0.1) {
        neck.neckIn();
    }
    else if (neckOut.getAsDouble() > 0.1) {
        neck.neckOut();
    }
    else {
      neck.neckOff();
    }

    // if(jawOpen.getAsBoolean()) {
    //   currentAngle = jaw.getJawAngle();
    //   jaw.jawOpen();
    // }
    // else if(jawClose.getAsBoolean() && !tongue.isLicked()) {
    //   currentAngle = jaw.getJawAngle();
    //   jaw.jawClose();
    // }
    // else {
    //   jaw.setJawAngle(currentAngle);
    // }

    // SmartDashboard.putBoolean("defaultNeck", true);
  
  }

  
  @Override
  public void end(boolean interrupted) {
    // neck.neckOff();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
