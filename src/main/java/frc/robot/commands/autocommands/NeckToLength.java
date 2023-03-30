// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.*;



public class NeckToLength extends CommandBase {
  /** Creates a new NeckToLength Command */

  private Neck neck;
  private double length;

  public NeckToLength(Neck neck, double length) {
    this.neck = neck;
    this.length = length;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(neck);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    neck.setNeckPosition(length);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(length - neck.getNeckDistance()) <= Units.inchesToMeters(3));    

    //4 is the radius of the interior of the cone
    //4 is a buffer zone for the robot -> 2 inches before, 2 inches past = 4 total inches
  }
}
