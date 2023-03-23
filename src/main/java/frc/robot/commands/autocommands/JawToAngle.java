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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.*;



public class JawToAngle extends CommandBase {
  /** Creates a new JawToAngle. */

  private Jaw jaw;
  private double angle;

  public JawToAngle(Jaw jaw, double angle) {
    this.jaw = jaw;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(jaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    jaw.setJawAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    jaw.setJawAngle(angle);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(angle - jaw.getJawAngle()) <= 2);
  }
}
