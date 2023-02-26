package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class LeftSideAuto extends SequentialCommandGroup{
    
    public LeftSideAuto(Swerve swerve){


        addCommands(
            // new firstConeAuto(swerve),
            // new secondConeAuto(swerve)
            new pathPlannerTest(swerve)
        );


    }




}
