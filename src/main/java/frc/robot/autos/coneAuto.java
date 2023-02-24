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

public class coneAuto extends SequentialCommandGroup {
    private String blockTrajJSON = "paths/MoveToFirstBlock.wpilib.json";
    private Trajectory blockTraj = new Trajectory();

    private String coneTrajJSON = "paths/MoveToFirstCone.wpilib.json";
    private Trajectory coneTraj = new Trajectory();

    private String pathPlanner1stConeJSON = "GetACone";
    private Trajectory planner1ConeTraj = new Trajectory();


    public coneAuto(Swerve s_Swerve){
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
            Path blockPath = Filesystem.getDeployDirectory().toPath().resolve(blockTrajJSON);
            blockTraj = TrajectoryUtil.fromPathweaverJson(blockPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blockTrajJSON, ex.getStackTrace());
        }
        
        try {
            Path conePath = Filesystem.getDeployDirectory().toPath().resolve(coneTrajJSON);
            coneTraj = TrajectoryUtil.fromPathweaverJson(conePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + coneTrajJSON, ex.getStackTrace());
        }
           

        PathPlannerTrajectory pathPlannerTest = PathPlanner.loadPath(pathPlanner1stConeJSON, new PathConstraints(4, 3));

        

        // try {
        //     Path planner1stCone = Filesystem.getDeployDirectory().toPath().resolve(pathPlanner1stConeJSON);
        //     planner1ConeTraj = TrajectoryUtil.fromPathweaverJson(conePath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory: " + coneTrajJSON, ex.getStackTrace());
        // }
        

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder pathPlannerBuilder = // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
            new SwerveAutoBuilder(
                s_Swerve::getPose, // Pose2d supplier
                s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDConstants(20, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(20, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
            );
        


        SwerveControllerCommand getConeAuto =
            new SwerveControllerCommand(
                coneTraj,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        

        SwerveControllerCommand getBlockAuto =
            new SwerveControllerCommand(
                blockTraj,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);



        addCommands(
            // new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
            // getConeAuto,
            // new InstantCommand(() -> s_Swerve.resetOdometry(pathPlannerTest.getInitialPose())),
            // pathPlannerBuilder.fullAuto(pathPlannerTest)
            
            
            new InstantCommand(() -> s_Swerve.resetOdometry(coneTraj.getInitialPose())),
            getConeAuto
        );
    }
}