package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.BalanceRobot;
import frc.robot.subsystems.Swerve;

public class BalanceAuto extends SequentialCommandGroup{
    
    public BalanceAuto(Swerve swerve){
        swerve.zeroGyro();
        swerve.resetOdometry(new Pose2d());
        double initAngle = swerve.getPitch();

        addCommands(
            new AutoDrive(-3, swerve, true, true),
            new AutoDrive(-5.2, swerve, true, false),
            new AutoTurn(180, swerve, true, true),
            new AutoDrive(3, swerve, true, true),
            new BalanceRobot(swerve, initAngle)
        );
    }

}
