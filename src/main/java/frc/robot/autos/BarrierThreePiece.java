package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.Grab;
import frc.robot.commands.autocommands.Lick;
import frc.robot.commands.autocommands.LimelightAlign;
import frc.robot.commands.autocommands.NeckToLength;
import frc.robot.commands.autocommands.LimelightAlign.PoleHeight;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.subsystems.Tongue;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tongue;

import java.io.IOException;
import java.nio.file.Path;
import java.time.InstantSource;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class BarrierThreePiece extends SequentialCommandGroup {

    public BarrierThreePiece(Swerve swerve, Jaw jaw, Tongue tongue, Neck neck, Grabber grabber){
        
        addRequirements(swerve, jaw, tongue, neck, grabber);

        // PathPlannerTrajectory gFCR = PathPlanner.loadPath("grabFirstConeReverseL", new PathConstraints(4.5, 5));
        // PathPlannerTrajectory.transformTrajectoryForAlliance(gFCR, DriverStation.getAlliance());
        // Command grabFirstConeReverse = swerve.followTrajectoryCommand(gFCR);
        // PathPlannerTrajectory.transformTrajectoryForAlliance(gFCS, DriverStation.getAlliance());


        PathPlannerTrajectory gFCS = PathPlanner.loadPath("barrierSideBlue", new PathConstraints(4, 4));
        PathPlannerTrajectory.transformTrajectoryForAlliance(gFCS, DriverStation.getAlliance());
        // PathPlannerTrajectory gFCS = PathPlanner.loadPath("barrierSide" + DriverStation.getAlliance().toString(), new PathConstraints(4, 4));
        Command threePieceAuto = swerve.followTrajectoryCommand(gFCS);

        SmartDashboard.putString("Drive Station Color", DriverStation.getAlliance().toString());


        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        eventMap.put("RetractNeck", new NeckToLength(neck, Constants.Snake.retractedLength));
        eventMap.put("LowerJaw", new JawToAngle(jaw, neck, Constants.Snake.downAngle));

        eventMap.put("OpenGrabber", new Grab(grabber, false, true));
        eventMap.put("GrabCone", new Grab(grabber, true, true));
        eventMap.put("OpenJaw", new JawToAngle(jaw, neck, Constants.Snake.midAngle));
        // eventMap.put("ExtendNeck", new NeckToLength(neck, Constants.Snake.highLength));

        
        FollowPathWithEvents fullPath = new FollowPathWithEvents(
            threePieceAuto,
            gFCS.getMarkers(),
            eventMap
        );


        
        addCommands(

            new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(() -> swerve.setOdometry(new Pose2d(gFCS.getInitialPose().getTranslation(), gFCS.getInitialHolonomicPose().getRotation()))),

            new Grab(grabber, true, false),

            new ParallelCommandGroup(
              new NeckToLength(neck, Constants.Snake.highLength),
              new JawToAngle(jaw, neck, Constants.Snake.highBlockAngle)  
            ),


            new Grab(grabber, false, true),
            
            new NeckToLength(neck, 0.7),

            new Grab(grabber, true, true),

            fullPath

            // new ParallelRaceGroup(
            //     new LimelightAlign(jaw, neck, swerve, PoleHeight.HIGH_POLE),
            //     new WaitCommand(1)
            // ),

            // new Grab(grabber, false, true)

        );

    }
}

