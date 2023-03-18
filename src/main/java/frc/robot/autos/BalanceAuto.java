package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.BalanceRobot;
import frc.robot.commands.autocommands.Lick;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.subsystems.Tongue;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tongue;

public class BalanceAuto extends SequentialCommandGroup{
    
    public BalanceAuto(Swerve swerve, Jaw jaw, Tongue tongue){
        swerve.zeroGyro();
        swerve.resetOdometry(new Pose2d());
        double initAngle = swerve.getPitch();

        addRequirements(swerve, jaw, tongue);

        addCommands(
            new JawToAngle(jaw, Constants.Snake.midAngle),
            new Lick(tongue, true),
            new AutoDrive(-3, swerve, true, true),
            new AutoDrive(-5.2, swerve, true, false),
            new AutoTurn(180, swerve, true, true),
            new AutoDrive(3, swerve, true, true),
            new BalanceRobot(swerve, initAngle)
        );
    }

}
