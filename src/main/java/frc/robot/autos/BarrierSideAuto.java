package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.PlaceCone;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.Boop;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.subsystems.BoopBoop;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;

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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class BarrierSideAuto extends SequentialCommandGroup {

    public BarrierSideAuto(Swerve swerve, Jaw jaw, BoopBoop booper, Neck neck, Grabber grabber){

        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerTrajectory gFCR = PathPlanner.loadPath("grabFirstConeReverseL", new PathConstraints(4.5, 5));
        Command grabFirstConeReverse = swerve.followTrajectoryCommand(gFCR);

        PathPlannerTrajectory gFCS = PathPlanner.loadPath("grabFirstConeSweepL", new PathConstraints(4.5, 5));
        Command grabFirstConeSweep = swerve.followTrajectoryCommand(gFCS);

        PathPlannerTrajectory gSCS = PathPlanner.loadPath("grabSecondConeSweepL", new PathConstraints(4.5, 5));
        Command grabSecondConeSweepCommand = swerve.followTrajectoryCommand(gSCS);

        PathPlannerTrajectory.transformTrajectoryForAlliance(gFCR, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(gFCS, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(gSCS, DriverStation.getAlliance());

        
        addCommands(

            new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(gFCS.getInitialPose().getTranslation(), Rotation2d.fromDegrees(180)))),
            new ParallelCommandGroup(
                new InstantCommand(() -> grabber.grabThang(false)),
                new JawToAngle(jaw, Constants.Snake.midAngle)
            ),
            new WaitCommand(0.1),
            new Boop(booper, true),
            new WaitCommand(0.2),


            // new InstantCommand(() -> booper.boop()),
            // new WaitCommand(0.2),
            // new InstantCommand(() -> booper.boop()),

            grabFirstConeSweep,

            new Boop(booper, true),
            new WaitCommand(0.2),

            // new WaitCommand(0.2),
            // new InstantCommand(() -> booper.boop()),
            // new WaitCommand(0.2),
            // new InstantCommand(() -> booper.boop()),
            
            grabSecondConeSweepCommand,

            new Boop(booper, true),
            new WaitCommand(0.2)
            
            // new WaitCommand(0.2),
            // new InstantCommand(() -> booper.boop()),
            // new WaitCommand(0.2),
            // new InstantCommand(() -> booper.boop())
        );

    }
}

