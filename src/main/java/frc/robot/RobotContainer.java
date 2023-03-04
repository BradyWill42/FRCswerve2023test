package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.*;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.autocommands.BalanceRobot;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    private final XboxController operator2 = new XboxController(1);

    /* Autonomous Chooser */
    private SendableChooser<Command> chooser;

    /* Drive Controls */
    private int translationAxis = XboxController.Axis.kLeftY.value;
    private int strafeAxis = XboxController.Axis.kLeftX.value;
    private int rotationAxis = XboxController.Axis.kRightX.value;

    /* Field Oriented Toggle */
    private boolean isFieldOriented = false;

    /* Booper Toggle */
    private boolean boopEnabled, grabEnabled;

    /* Driver Buttons */
    private final JoystickButton zeroOdometry = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton boop = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton balance = new JoystickButton(driver, XboxController.Button.)


    /* Operator Buttons */
    private final JoystickButton neckOut = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton neckIn = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton jawOpen = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton jawClose = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton toggleGrabber = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton resetEncoders = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton setJawPosition = new JoystickButton(operator, XboxController.Button.kX.value);
    

    private final JoystickButton midPos = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton lowPos = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton topPos = new JoystickButton(operator, XboxController.Button.kB.value);






    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Neck neck = new Neck();
    private final Jaw jaw = new Jaw();
    private final Grabber grabber = new Grabber();
    private final BoopBoop booper = new BoopBoop();

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
        

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> isFieldOriented
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
        zeroOdometry.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        robotCentric.toggleOnTrue(new InstantCommand(() -> toggleRobotCentric()));
        boop.onTrue(new InstantCommand(() -> toggleBooper()));

        /* Operator Buttons */
        (neckOut.onTrue(new InstantCommand(() -> neck.neckOut())).or(neckIn.onTrue(new InstantCommand(() -> neck.neckIn())))).onFalse(new InstantCommand(() -> neck.neckOff()));
        (jawOpen.onTrue(new InstantCommand(() -> jaw.jawOpen())).or(jawClose.onTrue(new InstantCommand(() -> jaw.jawClose())))).onFalse(new InstantCommand(() -> jaw.jawOff()));
        toggleGrabber.onTrue(new InstantCommand(() -> toggleGrabber()));
        resetEncoders.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> jaw.resetjawEncoder()),
                new InstantCommand(() -> neck.resetArmEncoders())
            )
        );
        setJawPosition.onTrue(new InstantCommand(() -> jaw.setJawAngle(Constants.Snake.midAngle)));
        
        // lowPos.onTrue(new PlaceCone(neck, jaw, Constants.Snake.downAngle, Constants.Snake.retractedLength));
        // midPos.onTrue(new PlaceCone(neck, jaw, Constants.Snake.midAngle, Constants.Snake.midLength));
        

        // if(pov == 180){
        //     // Mid position:
        //     new PlaceCone(neck, jaw, Constants.Snake.midAngle, Constants.Snake.midLength);   
        // }
        // else if(pov == 0){
        //     // Bottom position:
        //     new PlaceCone(neck, jaw, Constants.Snake.downAngle, Constants.Snake.retractedLength);   
        // }


    }

    public void toggleRobotCentric(){
        isFieldOriented = !isFieldOriented;
    }

    public void toggleBooper(){
        boopEnabled = !boopEnabled;
        booper.pump(boopEnabled);
    }

    public void toggleGrabber(){
        grabEnabled = !grabEnabled;
        grabber.grabThang(grabEnabled);
    }


    public void initializeAutoChooser() {
        chooser.setDefaultOption("Nothing", new AutoTurn(180, s_Swerve, true, true));

        chooser.addOption("Balance Auto", new BalanceAuto(s_Swerve, jaw, booper));

        chooser.addOption("Score from Barrier Side", new BarrierSideAuto(s_Swerve, jaw, booper, neck, grabber));

        chooser.addOption("Score from Wall Side", new WallSideAuto(s_Swerve, jaw, booper, neck, grabber));

        chooser.addOption("Set Arm to 65 Degrees", new InstantCommand(() -> jaw.setJawAngle(Constants.Snake.midAngle)));

        chooser.addOption("Boop Test", new BoopAuto(booper, grabber, jaw));
        // chooser.addOption("Score from Right Side", getAutonomousCommand());

        SmartDashboard.putData(chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}



//I think the programmer likes animals a little too much
