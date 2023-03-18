package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.Lick;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Jaw;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tongue;

import java.io.File;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class WallSideAuto extends SequentialCommandGroup {

    public WallSideAuto(Swerve swerve, Jaw jaw, Tongue tongue, Neck neck, Grabber grabber){

        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // PathPlannerTrajectory gFCR = PathPlanner.loadPath("grabFirstConeReverseR", new PathConstraints(4, 3));
        // Command grabFirstConeReverse = swerve.followTrajectoryCommand(gFCR);

        PathPlannerTrajectory gFCS = PathPlanner.loadPath("grabFirstConeSweepR", new PathConstraints(4.5, 5));
        PathPlannerTrajectory.transformTrajectoryForAlliance(gFCS, DriverStation.getAlliance());
        Command grabFirstConeSweep = swerve.followTrajectoryCommand(gFCS);

        PathPlannerTrajectory gSCS = PathPlanner.loadPath("grabSecondConeSweepR", new PathConstraints(4.5, 5));
        PathPlannerTrajectory.transformTrajectoryForAlliance(gSCS, DriverStation.getAlliance());
        Command grabSecondConeSweepCommand = swerve.followTrajectoryCommand(gSCS);


        
        addCommands(

            new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(gFCS.getInitialPose().getTranslation(), Rotation2d.fromDegrees(180)))),
            new Lick(tongue, false),
            // new ParallelCommandGroup(
            //     new InstantCommand(() -> jaw.resetjawEncoder()),
            //     new InstantCommand(() -> neck.resetArmEncoders())
            // ),

            // new InstantCommand(() -> grabber.grab()),
            // new PlaceCone(neck, jaw, Constants.Snake.midAngle, Constants.Snake.midLength),
            // new InstantCommand(() -> grabber.grab()),
            // new PlaceCone(neck, jaw, Constants.Snake.midAngle, Constants.Snake.retractedLength),
            new JawToAngle(jaw, Constants.Snake.midAngle),

            new Lick(tongue, true),
            new WaitCommand(0.2),
            new Lick(tongue, false),


            // new InstantCommand(() -> tongue.boop()),
            // new WaitCommand(0.2),
            // new InstantCommand(() -> tongue.boop()),

            grabFirstConeSweep,

            new Lick(tongue, true),
            new WaitCommand(0.2),
            new Lick(tongue, false),

            // new WaitCommand(0.2),
            // new InstantCommand(() -> tongue.boop()),
            // new WaitCommand(0.2),
            // new InstantCommand(() -> tongue.boop()),
            
            grabSecondConeSweepCommand,

            new Lick(tongue, true),
            new WaitCommand(0.2),
            new Lick(tongue, false)
            
            // new WaitCommand(0.2),
            // new InstantCommand(() -> tongue.boop()),
            // new WaitCommand(0.2),
            // new InstantCommand(() -> tongue.boop())
        );

    }
}

