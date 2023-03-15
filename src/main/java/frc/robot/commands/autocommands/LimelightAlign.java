package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;


public class LimelightAlign extends CommandBase{
  private Jaw jaw;
  private Neck neck;
  private Swerve swerve;
  private Limelight limelight;
  private double horizontalDist, verticalDist;

  
  public LimelightAlign(Jaw jaw, Neck neck, Swerve swerve, Limelight Limelight) {
    
    this.limelight = limelight;

    this.jaw = jaw;
    this.neck = neck;
    this.swerve = swerve;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(jaw, neck, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
