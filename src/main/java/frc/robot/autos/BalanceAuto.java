package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.BalanceRobot;
import frc.robot.commands.autocommands.Grab;
import frc.robot.commands.autocommands.Lick;
import frc.robot.commands.autocommands.NeckToLength;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.subsystems.Tongue;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tongue;

public class BalanceAuto extends SequentialCommandGroup{
    
    public BalanceAuto(Swerve swerve, Jaw jaw, Tongue tongue, Neck neck, Grabber grabber){
        swerve.zeroGyro();
        swerve.resetOdometry();
        double initAngle = swerve.getRoll();

        addRequirements(swerve, jaw, tongue);

        addCommands(

            new Grab(grabber, true, false),

            new ParallelCommandGroup(
              new NeckToLength(neck, Constants.Snake.highLength),
              new JawToAngle(jaw, neck, Constants.Snake.highBlockAngle)  
            ),

            new Grab(grabber, false, true),
            
            new NeckToLength(neck, 0),

            new JawToAngle(jaw, neck, Constants.Snake.midAngle),

            new AutoDrive(-3, swerve, true, true),
            new AutoDrive(-5, swerve, true, false),
            new AutoDrive(2.5, swerve, true, true),


            new BalanceRobot(swerve, initAngle)

        );
    }

}
