package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.commands.autocommands.NeckToLength;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;




public class PlaceCone extends ParallelCommandGroup{
  public PlaceCone(Neck neck, Jaw jaw, double angle, double length){
    
    addCommands(
      new ParallelCommandGroup(
        new JawToAngle(jaw, angle),
        new NeckToLength(neck, length)
      )
    );
  }
}
