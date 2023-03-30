package frc.robot.commands.autocommands;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LightMode;


public class LimelightAlign extends CommandBase{
  private Jaw jaw;
  private Neck neck;
  private Swerve swerve;
  private double angleToPole, distanceToBase, lengthToPole, distanceFromTarget, strafeOffset;
  public enum PoleHeight{
    GROUND, MID_POLE, HIGH_POLE
  }
  public PoleHeight poleHeight;

  
  public LimelightAlign(Jaw jaw, Neck neck, Swerve swerve, PoleHeight poleHeight) {
    
    this.jaw = jaw;
    this.neck = neck;
    this.swerve = swerve;
    this.poleHeight = poleHeight;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.''
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);
    swerve.setOdometry(new Pose2d());    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Limelight X", Limelight.getTx());

    switch(poleHeight){

      //Ground case uses mid height pole to align
      case GROUND:
        angleToPole = Math.toRadians(Constants.Snake.limelightAngle + Limelight.getTy());
        distanceToBase = (Constants.Snake.midPoleHeight - Constants.Snake.limelightHeight) / Math.tan(Constants.Snake.limelightAngle);
        lengthToPole = Constants.Snake.lengthToMidPole;
        SmartDashboard.putString("Pole Selected: ", "Ground");
        break;

      //Mid Case uses mid height pole to align
      case MID_POLE:
      angleToPole = Math.toRadians(Constants.Snake.limelightAngle + Limelight.getTy());
      distanceToBase = (Constants.Snake.midPoleHeight - Constants.Snake.limelightHeight) / Math.tan(angleToPole);
      lengthToPole = Constants.Snake.lengthToMidPole;
      SmartDashboard.putString("Pole Selected: ", "Mid Pole");
      SmartDashboard.putNumber("AngleToPole", Math.toDegrees(angleToPole));
      break;

      //High Case uses high height pole to align
      case HIGH_POLE:
      angleToPole = Math.toRadians(Constants.Snake.limelightAngle + Limelight.getTy());
      distanceToBase = (Constants.Snake.highPoleHeight - Constants.Snake.limelightHeight) / Math.tan(angleToPole);
      lengthToPole = Constants.Snake.lengthToHighPole;
      SmartDashboard.putString("Pole Selected: ", "High Pole");
      break;

    }
    //Calculates distance robot has to drive forward to be on the edge of the scoring station
    distanceFromTarget = distanceToBase - lengthToPole;

    //calculates how far left or right we have to strafe to align with the pole we are scoring on. 
    strafeOffset = distanceToBase * Math.tan(Math.toRadians(-Limelight.getTx()));


    SmartDashboard.putNumber("distanceFromTarget", distanceFromTarget);
    SmartDashboard.putNumber("strafeOffset", strafeOffset);

    if(Limelight.isTarget()){
      swerve.drive(new Translation2d(distanceFromTarget, strafeOffset).times(8.0), 0, true, true);
    }   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Limelight.setLedMode(LightMode.eOff);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(distanceFromTarget) < 0.01 && Math.abs(strafeOffset) < 0.01){
      return true;
    } else {
      return false;
    }
  }
}
