package frc.robot.autos;

import java.util.List;

import javax.xml.namespace.QName;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Balance extends SequentialCommandGroup{
    private double stationWidth = 1;
    private double stationLength = 1;


    private Trajectory forward;
    private Trajectory backward;
    private Trajectory zero;
    private Trajectory finTraj;

     

    private double currentPitch;

    private Swerve swerve;

    public Balance(Swerve swerve) {

        TrajectoryConfig config =
            new TrajectoryConfig(
                    1,
                    1)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory forward =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of(
                    new Pose2d(new Translation2d(0.1,0), new Rotation2d(0))
                    //new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
                ),
                config);

        Trajectory backward =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of(
                    new Pose2d(new Translation2d(-0.1,0), new Rotation2d(0))
                    //new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
                ),
                config);
        
        Trajectory zero =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of(
                    new Pose2d(new Translation2d(0,0), new Rotation2d(0))
                    //new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
                ),
                config);       
        
        

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d())),
            trajToRun()
        );


    }

    public int isBalanced(){
        if(swerve.getPitch() < -3){
            return -1;
        } else if(swerve.getPitch() > 3){
            return 1;
        } else {
            return 0;
        }
    }

    public Command trajToRun(){

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        

        SwerveControllerCommand balanceForward =
            new SwerveControllerCommand(
                forward,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);
        SwerveControllerCommand balanceBackward =
            new SwerveControllerCommand(
                backward,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        SwerveControllerCommand balanced =
            new SwerveControllerCommand(
                zero,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        while(isBalanced() != 0){
            if(isBalanced() == -1){
                return balanceForward;
            } else if(isBalanced() == 1){
                return balanceBackward;
            }
        }
        return balanced;

        
    }



}
