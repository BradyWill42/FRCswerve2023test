package frc.robot;

import java.util.function.BooleanSupplier;

import javax.xml.crypto.dsig.spec.HMACParameterSpec;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.BalanceRobot;
import frc.robot.commands.defaultcommands.DefaultHead;
import frc.robot.commands.defaultcommands.DefaultMouth;
import frc.robot.commands.defaultcommands.DefaultSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Autonomous Chooser */
    private SendableChooser<Command> chooser;

    /* Event Loop */
    private final EventLoop eventLoop = new EventLoop();

    /* Drive Controls */
    private int translationAxis = XboxController.Axis.kLeftY.value;
    private int strafeAxis = XboxController.Axis.kLeftX.value;
    private int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Controls */
    private int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;

    /* Field Oriented Toggle */
    private boolean isFieldOriented = false;

    /* Licker Toggle */
    private boolean lick = false, coneGrab = false, cubeGrab = false;

    /* Driver Buttons */
    private final JoystickButton zeroOdometry = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton boop = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton balance = new JoystickButton(driver, XboxController.Button.)


    /* Operator Buttons */
    private final JoystickButton neckOut = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton neckIn = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton jawOpen = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton jawClose = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton resetEncoders = new JoystickButton(operator, XboxController.Button.kY.value); 
    private final JoystickButton tongueLick = new JoystickButton(operator, XboxController.Button.kB.value);



    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Neck neck = new Neck();
    private final Jaw jaw = new Jaw();
    private final Grabber grabber = new Grabber();
    private final Tongue tongue = new Tongue();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if(translationAxis < Math.abs(0.1)){
            translationAxis = 0;
        }
        if(strafeAxis < Math.abs(0.1)){
            strafeAxis = 0;
        }
        if(rotationAxis < Math.abs(0.1)){
            rotationAxis = 0;
        }      
        
        if(leftTriggerAxis < Math.abs(0.1)){
            leftTriggerAxis = 0;
        }
        if(rightTriggerAxis < Math.abs(0.1)){
            rightTriggerAxis = 0;
        }        
        

        swerve.setDefaultCommand(
            new DefaultSwerve(
                swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> isFieldOriented
                )
        );

        jaw.setDefaultCommand(
            new DefaultHead(
                () -> neckIn.getAsBoolean(), 
                () -> neckOut.getAsBoolean(), 
                () -> jawOpen.getAsBoolean(), 
                () -> jawClose.getAsBoolean(), 
                jaw, 
                neck, 
                tongue, 
                grabber
            )
        );
        neck.setDefaultCommand(
            new DefaultHead(
                () -> neckIn.getAsBoolean(), 
                () -> neckOut.getAsBoolean(), 
                () -> jawOpen.getAsBoolean(), 
                () -> jawClose.getAsBoolean(), 
                jaw, 
                neck, 
                tongue, 
                grabber
            )
        );
        grabber.setDefaultCommand(
            new DefaultMouth(
                () -> lick, 
                () -> coneGrab, 
                () -> cubeGrab, 
                jaw, 
                neck, 
                tongue, 
                grabber
            )
        );
        tongue.setDefaultCommand(
            new DefaultMouth(
                () -> lick, 
                () -> coneGrab, 
                () -> cubeGrab, 
                jaw, 
                neck, 
                tongue, 
                grabber
            )
        );     

    
        //Initalize Autonomous Chooser
        chooser = new SendableChooser<Command>();

        // Configure the button bindings
        configureButtonBindings();

        //Initialize Autonomous Chooser
        initializeAutoChooser();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroOdometry.onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        robotCentric.toggleOnTrue(new InstantCommand(() -> toggleRobotCentric()));



        /* Operator Buttons */
        tongueLick.toggleOnTrue(new InstantCommand(() -> toggleLicker()));

        resetEncoders.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> jaw.resetjawEncoder()),
                new InstantCommand(() -> neck.resetArmEncoders())
            )
        );

    }

    public void toggleRobotCentric(){
        isFieldOriented = !isFieldOriented;
    }

    public void toggleLicker(){
        lick = !lick;
        // tongue.lick(lick);
    }

    public void toggleGrabCone(){
        coneGrab = !coneGrab;
        // grabber.grabThang(coneGrab);
    }

    public void toggleGrabCube(){
        cubeGrab = !cubeGrab;
    }

    public void initializeAutoChooser() {
        chooser.setDefaultOption("Nothing", new AutoTurn(180, swerve, true, true));

        chooser.addOption("Balance Auto", new BalanceAuto(swerve, jaw, tongue));

        chooser.addOption("Score from Barrier Side", new BarrierSideAuto(swerve, jaw, tongue, neck, grabber));

        chooser.addOption("Score from Wall Side", new WallSideAuto(swerve, jaw, tongue, neck, grabber));

        chooser.addOption("Set Arm to 65 Degrees", new InstantCommand(() -> jaw.setJawAngle(Constants.Snake.midAngle)));

        chooser.addOption("Lick Test", new LickAuto(tongue, grabber, jaw));

        chooser.addOption("Lick and Drive Forward", new DriveForward(swerve, jaw, tongue, neck, grabber));
        // chooser.addOption("Score from Right Side", getAutonomousCommand());

        SmartDashboard.putData(chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // SmartDashboard.putNumber("PSI", new Compressor(PneumaticsModuleType.CTREPCM).getPressure())
        return chooser.getSelected();
    }
}



//I think the programmer likes animals a little too much
