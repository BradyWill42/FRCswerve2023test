package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoShoot;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.defaultcommands.DefaultIntake;
import frc.robot.commands.defaultcommands.DefaultRotateTurret;
import frc.robot.commands.defaultcommands.DefaultShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;



public class PlaceCone extends ParallelCommandGroup{
  public PlaceCone(Neck neck, Jaw jaw, double angle, double length){
    
    addCommands(
      new InstantCommand(() -> neck.setNeckPosition(length));
      new InstantCommand(() -> jaw.setJawAngle(angle));
    );
      
  }
}
