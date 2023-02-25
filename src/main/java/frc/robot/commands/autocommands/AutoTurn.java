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



public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */

  private Swerve drivetrain;
  private Rotation2d rotation;
  private boolean isOpenLoop, fieldRelative;

  public AutoTurn(Rotation2d rotation, Swerve drivetrain,boolean fieldRelative, boolean isOpenLoop) {
    this.rotation = rotation;
    this.isOpenLoop = isOpenLoop;
    this.fieldRelative = fieldRelative;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivetrain.resetOdometry(new Pose2d());
    drivetrain.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new Translation2d(0, 0), rotation.getRadians(), fieldRelative, isOpenLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("You Reached This", "rotatedIdiot");
    drivetrain.drive(new Translation2d(), 0, false, true);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    SmartDashboard.putNumber("currentRotationalpose", drivetrain.getPose().getRotation().getDegrees());

    return (Math.abs(rotation.getDegrees() - drivetrain.getPose().getRotation().getDegrees()) <= 5);

  }
}
