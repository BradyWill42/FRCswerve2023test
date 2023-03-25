package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.Lick;
import frc.robot.commands.autocommands.NeckToLength;
import frc.robot.commands.autocommands.JawToAngle;
import frc.robot.subsystems.Tongue;
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


public class DriveForward extends SequentialCommandGroup {

    public DriveForward(Swerve swerve, Jaw jaw, Tongue tongue, Neck neck, Grabber grabber){

        

        PathPlannerTrajectory dF = PathPlanner.loadPath("driveForward", new PathConstraints(3, 3));
        
        PathPlannerTrajectory.transformTrajectoryForAlliance(dF, DriverStation.getAlliance());

        Command driveForwardCommand = swerve.followTrajectoryCommand(dF);

        SmartDashboard.putNumber("Initial Rotation", dF.getInitialHolonomicPose().getRotation().getDegrees());
        
        

        
        addCommands(

            new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(dF.getInitialPose().getTranslation(), dF.getInitialHolonomicPose().getRotation()))),
            
            
            new InstantCommand(() -> grabber.grabThang(false)),
            new WaitCommand(0.2),

            new ParallelCommandGroup(
                new JawToAngle(jaw, neck, Constants.Snake.midAngle)
            ),
                    
            new Lick(tongue, true),
            new WaitCommand(0.2),

            driveForwardCommand

        );

    }
}

