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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.*;



public class Lick extends CommandBase {
  /** Creates a new Lick Command. */

  private Tongue tongue;
  private boolean lickEnabled;
  private Timer timer;


  public Lick(Tongue tongue, boolean lickEnabled) {
    this.tongue = tongue;
    this.lickEnabled = lickEnabled;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tongue);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tongue.lick(false);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tongue.lick(lickEnabled);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tongue.lick(false);
    new WaitCommand(0.2);
    timer.stop();
    timer.reset();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((tongue.isLicked() == lickEnabled) && timer.hasElapsed(0.2));
  }
}
