package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

        addRequirements(swerve, jaw, tongue, neck, grabber);

        // PathPlannerTrajectory gFCR = PathPlanner.loadPath("grabFirstConeReverseL", new PathConstraints(4.5, 5));
        // PathPlannerTrajectory.transformTrajectoryForAlliance(gFCR, DriverStation.getAlliance());
        // Command grabFirstConeReverse = swerve.followTrajectoryCommand(gFCR);
        // PathPlannerTrajectory.transformTrajectoryForAlliance(gFCS, DriverStation.getAlliance());


        PathPlannerTrajectory balanceAuto = PathPlanner.loadPath("BalanceRobot", new PathConstraints(4, 4));
        PathPlannerTrajectory.transformTrajectoryForAlliance(balanceAuto, DriverStation.getAlliance());
        // PathPlannerTrajectory gFCS = PathPlanner.loadPath("barrierSide" + DriverStation.getAlliance().toString(), new PathConstraints(4, 4));
        Command setUpBalance = swerve.followTrajectoryCommand(balanceAuto);

        SmartDashboard.putString("Drive Station Color", DriverStation.getAlliance().toString());


        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        eventMap.put("BalanceRobot", new BalanceRobot(swerve, initAngle));
        // eventMap.put("ExtendNeck", new NeckToLength(neck, Constants.Snake.highLength));

        
        FollowPathWithEvents fullPath = new FollowPathWithEvents(
            setUpBalance,
            balanceAuto.getMarkers(),
            eventMap
        );


        
        addCommands(

            new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(() -> swerve.setOdometry(new Pose2d(balanceAuto.getInitialPose().getTranslation(), balanceAuto.getInitialHolonomicPose().getRotation()))),

            new Grab(grabber, true, false),

            new ParallelCommandGroup(
              new NeckToLength(neck, Constants.Snake.highLength),
              new JawToAngle(jaw, neck, Constants.Snake.highBlockAngle)  
            ),


            new Grab(grabber, false, true),
            
            new NeckToLength(neck, 0),
            new JawToAngle(jaw, neck, Constants.Snake.downAngle),
            
            fullPath

            // new ParallelRaceGroup(
            //     new LimelightAlign(jaw, neck, swerve, PoleHeight.HIGH_POLE),
            //     new WaitCommand(1)
            // ),

            // new Grab(grabber, false, true)

        );
    }

}
