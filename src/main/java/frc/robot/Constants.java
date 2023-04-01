package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSNeoSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSNeoSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSNeoSwerveConstants.SDSMK4(COTSNeoSwerveConstants.driveGearRatios.SDSMK4_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.5715; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.5715; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive neo to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */

        //Align with bevel gears on right side

        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            // public static final int driveMotorID = 2;
            // public static final int angleMotorID = 1;
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(264.8144);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            // public static final int driveMotorID = 8;
            // public static final int angleMotorID = 7;
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(51.6796);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            // public static final int driveMotorID = 4;
            // public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(206.1035);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            // public static final int driveMotorID = 6;
            // public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(138.0761);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Snake {

        /* Snake Neutral Modes */
        public static final IdleMode rightNeckNeutralMode = IdleMode.kBrake;
        public static final IdleMode leftNeckNeutralMode = IdleMode.kBrake;
        public static final IdleMode jawNeutralMode = IdleMode.kBrake;

        /* Snake Neck Motor Constants */
        public static final int rightNeckMotorID = 9;
        public static final int leftNeckMotorID = 10;
        public static final int jawMotorID = 11;

        /* Snake Encoder DIO Ports */
        public static final int rightNeckEncoderID1 = 0;
        public static final int rightNeckEncoderID2 = 1;
        public static final int leftNeckEncoderID1 = 2;
        public static final int leftNeckEncoderID2 = 3;
        public static final int jawEncoderID1 = 4;
        public static final int jawEncoderID2 = 5;
        public static final int limitSwitchID = 6;

        /* Snake Jaw Angle Gear Ratio */
        public static final double jawGearRatio = (100.0 / 1.0);

        /* Snake Neck Gear Ratio */
        public static final double neckGearRatio = (10.62 / 1.0);

        /* Pneumatics Constants */
        public static final int brakeID1 = 6;
        public static final int brakeID2 = 7;

        public static final int boopID1 = 3;
        public static final int boopID2 = 2;

        public static final int grabberID1 = 1;
        public static final int grabberID2 = 0;

        public static final int changePressureID1 = 4;
        public static final int changePressureID2 = 5;

        /* Jaw and Neck Distance and Angle Constants */
        public static final double downAngle = 0.0;
        public static final double autoAngle = 45.0;
        public static final double midAngle = 45.0;
        public static final double highConeAngle = 48.0;
        public static final double highBlockAngle = 45.0;
        public static final double midLength = 1.25;
        public static final double highLength = 1.6;
        public static final double retractedLength = 0.0001;

        /* Limelight Constants */
        public static final double limelightAngle = 30.0;
        public static final double limelightHeight = 0.338;
        public static final double midPoleTapeHeight = 0.622;
        public static final double midPoleHeight = 0.87;
        public static final double highPoleHeight = 1.17;
        public static final double lengthToMidPole = 0.55;
        public static final double lengthToHighPole = 1.4;

        /* Jaw PID Constants */
        public static final double jawPP = 0.025;
        public static final double jawPI = 0.001;
        public static final double jawPD = 0.001;
        public static final double jawPF = 0.0;
        public static final int jawPSlot = 0;

        public static final double jawVP = 0.0;
        public static final double jawVI = 0.0;
        public static final double jawVD = 0.0;
        public static final double jawVF = 0.0;
        public static final int jawVSlot = 1;

        /* Neck PID Constants */
        public static final double neckPP = 3.596;
        public static final double neckPI = 0.0;
        public static final double neckPD = 0.0;
        public static final double neckPF = 0.0;
        public static final int neckPSlot = 0;

        public static final double neckVP = 0.0;
        public static final double neckVI = 0.0;
        public static final double neckVD = 0.0;
        public static final double neckVF = 0.0;
        public static final int neckVSlot = 1;

    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 20;
        public static final double kPYController = 20;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
