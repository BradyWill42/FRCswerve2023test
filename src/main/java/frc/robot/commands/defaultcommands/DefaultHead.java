// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

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
  private BooleanSupplier neckIn, neckOut, jawOpen, jawClose;

  public DefaultHead(BooleanSupplier neckIn, BooleanSupplier neckOut, BooleanSupplier jawOpen, BooleanSupplier jawClose, Jaw jaw, Neck neck, Tongue tongue, Grabber grabber) {
    this.jaw = jaw;
    this.neck = neck;
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
    if(neckIn.getAsBoolean()) {
      neck.neckIn();
    }
    else if (neckOut.getAsBoolean()) {
        neck.neckOut();
    }
    else {
      neck.neckOff();
    }

    if(jawOpen.getAsBoolean()) {
      jaw.jawOpen();
    }
    else if(jawClose.getAsBoolean()) {
      jaw.jawClose();
    }
    else {
      jaw.jawOff();
    }
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
