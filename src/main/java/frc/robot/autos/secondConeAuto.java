package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SecondConeAuto extends SequentialCommandGroup {

    private String secondConeTrajJSON = "paths/MoveToSecondCone.wpilib.json";
    private Trajectory secondConeTraj = new Trajectory();

    public SecondConeAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of(
                    new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                    new Pose2d(new Translation2d(1,0), new Rotation2d(0))
                    //new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
                ),
                config);

        SmartDashboard.putNumber("Initial XPose", exampleTrajectory.getInitialPose().getX());
        SmartDashboard.putNumber("Initial YPose", exampleTrajectory.getInitialPose().getY());

        
        try {
            Path secondConePath = Filesystem.getDeployDirectory().toPath().resolve(secondConeTrajJSON);
            secondConeTraj = TrajectoryUtil.fromPathweaverJson(secondConePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + secondConeTrajJSON, ex.getStackTrace());
        }

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);     


        SwerveControllerCommand getsecondConeAuto =
            new SwerveControllerCommand(
                secondConeTraj,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            // new InstantCommand(() -> s_Swerve.resetOdometry(secondConeTraj.getInitialPose())),
            getsecondConeAuto
        );
    }
}