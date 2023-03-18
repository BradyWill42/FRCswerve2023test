package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;


public class LimelightAlign extends CommandBase{
  private Jaw jaw;
  private Neck neck;
  private Swerve swerve;
  private Limelight limelight;
  private double horizontalDist, verticalDist, angleToPole, distanceToBase;
  public enum PoleHeight{
    GROUND, MID_POLE, HIGH_POLE
  }
  public PoleHeight poleHeight;

  
  public LimelightAlign(Jaw jaw, Neck neck, Swerve swerve, Limelight Limelight, PoleHeight poleHeight) {
    
    this.limelight = limelight;

    this.jaw = jaw;
    this.neck = neck;
    this.swerve = swerve;
    this.poleHeight = poleHeight;
    
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(jaw, neck, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(poleHeight){
     
      case MID_POLE:
      angleToPole = (Constants.Snake.limelightAngle + limelight.getY()) * (Math.PI / 180);
      distanceToBase = (Constants.Snake.midPoleHeight - Constants.Snake.limelightHeight) / Math.tan(angleToPole);
      SmartDashboard.putString("Pole Selected: ", "Mid Pole");
      break;

      case HIGH_POLE:
      angleToPole = (Constants.Snake.limelightAngle + limelight.getY()) * (Math.PI / 180);
      distanceToBase = (Constants.Snake.highPoleHeight - Constants.Snake.limelightHeight) / Math.tan(angleToPole);
      SmartDashboard.putString("Pole Selected: ", "High Pole");
      break;

      case GROUND:
      angleToPole = (Constants.Snake.limelightAngle + limelight.getY()) * (Math.PI / 180);
      distanceToBase = (0 - Constants.Snake.limelightHeight) / Math.tan(Constants.Snake.limelightAngle);
      SmartDashboard.putString("Pole Selected: ", "Ground");
      break;

    }
    
    // if(targetHeight == MID_POLE){
    // }

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
