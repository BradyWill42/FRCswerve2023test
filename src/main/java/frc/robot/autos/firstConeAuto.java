package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class FirstConeAuto extends SequentialCommandGroup {
    private String firstConeTrajJSON = "paths/MoveToFirstCone.wpilib.json";
    private Trajectory firstConeTraj = new Trajectory();


    public FirstConeAuto(Swerve s_Swerve){
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
            Path firstConePath = Filesystem.getDeployDirectory().toPath().resolve(firstConeTrajJSON);
            firstConeTraj = TrajectoryUtil.fromPathweaverJson(firstConePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + firstConeTrajJSON, ex.getStackTrace());
        }

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand getfirstConeAuto =
            new SwerveControllerCommand(
                firstConeTraj,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);



        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstConeTraj.getInitialPose())),
            getfirstConeAuto
        );
    }
}

