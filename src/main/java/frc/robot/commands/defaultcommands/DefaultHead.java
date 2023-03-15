// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.plaf.ToolBarUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Tongue;

public class DefaultHead extends CommandBase {
  
  private Jaw jaw;
  private Neck neck;
  private Tongue tongue;
  private Grabber grabber;
  private BooleanSupplier jawOpen, jawClose;
  private DoubleSupplier neckIn, neckOut;

  public DefaultHead(DoubleSupplier neckIn, DoubleSupplier neckOut, BooleanSupplier jawOpen, BooleanSupplier jawClose, Jaw jaw, Neck neck, Tongue tongue, Grabber grabber) {
    this.jaw = jaw;
    this.neck = neck;
    this.tongue = tongue;
    this.grabber = grabber;

    addRequirements(jaw, neck);

    this.neckIn = neckIn;
    this.neckOut = neckOut;
    this.jawOpen = jawOpen;
    this.jawClose = jawClose;
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(neckIn.getAsDouble() > 0.1 && jaw.getJawAngle() > 60) {
        neck.neckIn();
    }
    else if (neckOut.getAsDouble() > 0.1 && jaw.getJawAngle() > 60) {
        neck.neckOut();
    }
    else {
      neck.neckOff();
    }

    if(jawOpen.getAsBoolean()) {
      jaw.jawOpen();
    }
    else if(jawClose.getAsBoolean() && !tongue.isLicked()) {
        jaw.jawClose();
    }
    else {
      jaw.jawOff();
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    jaw.jawOff();
    neck.neckOff();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
