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



public class BalanceRobot extends CommandBase {
  /** Creates a new AutoDrive. */

  private Swerve drivetrain;

  public BalanceRobot(Swerve drivetrain) {
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
    if(isBalanced() == -1){
      new AutoDrive(0.1, drivetrain, false);
    } else if(isBalanced() == 1){
        new AutoDrive(-0.1, drivetrain, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new AutoDrive(0, drivetrain, false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(isBalanced() == 0){
      return true;
    } else {
      return false;
    }

  }

  public int isBalanced(){
    if(drivetrain.getPitch() < -2){
        return -1;
    } else if(drivetrain.getPitch() > 2){
        return 1;
    } else {
        return 0;
    }
}

}
