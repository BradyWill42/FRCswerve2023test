// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.*;



public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */

  private Swerve drivetrain;
  private double distanceX;
  private boolean fieldRelative;


  public AutoDrive(double distancex, Swerve drivetrain, boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
    this.distanceX = distancex;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fieldRelative){
      drivetrain.drive(new Translation2d(distanceX, 0), 0, true, true);
    } else {
      drivetrain.drive(new Translation2d(distanceX, 0), 0, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("You Reached This", "fIX1");
    drivetrain.drive(new Translation2d(), 0, false, true);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    SmartDashboard.putNumber("currentXpose", drivetrain.getPose().getX());
    SmartDashboard.putNumber("currentYpose", drivetrain.getPose().getY());

    if(Math.abs(distanceX) <= Math.abs(drivetrain.getPose().getX())){
        return true;
    } else {
      return false;
    }
  }
}
