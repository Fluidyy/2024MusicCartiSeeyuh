// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.intakesub;
import java.io.File;

import javax.swing.Box;
import javax.swing.plaf.basic.BasicTreeUI.TreePageAction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.intakesub;

import frc.robot.subsystems.lookuptable.lookuptable;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Boxpiv;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.projectiles;
// import frc.robot.subsystems.outake;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    


  // The robot's subsystems and c ommands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final Limelight limelight = new Limelight();



  private final intakesub intakesub = new intakesub();
  private final Boxpiv boxpivsub = new Boxpiv();
 private final Climb climbsub = new Climb();
  private final projectiles projectilesub = new projectiles();
  private final PhotonVision vision = new PhotonVision();
  // private final PhotonVision Vision = new PhotonVision();


                                       
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  Joystick operator = new Joystick(1);

  private final SendableChooser<Command> autoChooser;




  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driver = new XboxController(0);

  private final JoystickButton driver_limelightButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

  private final JoystickButton climbButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton climbPivButton = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton speakerButton = new JoystickButton(operator, XboxController.Button.kA.value);
  //private final JoystickButton intakehmanplayerButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton AutoaimSpeaker = new JoystickButton(operator, XboxController.Button.kX.value);
   private final JoystickButton AMPButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton TrapButton = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton intakehmanplayerButton= new JoystickButton(operator,XboxController.Button.kRightStick.value);
  private final JoystickButton UnjamButton= new JoystickButton(driver,XboxController.Button.kY.value);











  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()



  {
    var visionEst = vision.getEstimatedGlobalPose();
  visionEst.ifPresent(
          est -> {
              var estPose = est.estimatedPose.toPose2d();
              // Change our trust in the measurement based on the tags we can see
              var estStdDevs = vision.getEstimationStdDevs(estPose);

              drivebase.addVisionMeasurement(
                     
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });

    



    NamedCommands.registerCommand("intakedown", intakesub.intakepid(10));
  //  NamedCommands.registerCommand("peiceintake", new ParallelCommandGroup(intakesub.intakeCommand(0.3),intakesub.feederCommand()));
    //Make Multiple of the below shoot speakers because you need to manually find the setpoints and speed for each angle
    NamedCommands.registerCommand("shootS", new SequentialCommandGroup(boxpivsub.boxpivcmdTO(10),projectilesub.Outtake(.5)));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    drivebase.odometrygetshuffleboard();

    configureBindings();
   
    projectilesub.shuffleboard();
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driver.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driver.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driver.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driver::getYButtonPressed,
                                                                   driver::getAButtonPressed,
                                                                   driver::getXButtonPressed,
                                                                   driver::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRightX(),
        () -> driver.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(2));

    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    teleop closedFieldRelOperator = new teleop(
      drivebase,
      () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),

      () -> driver.getRightX(), () -> true);

    drivebase.setDefaultCommand(closedFieldRelOperator);
    




  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings()
  {
    /* 
    Itake Button DONE
    
    SPEAKER SHOOT  DONE

    Intake from Human Player DONE

    Climb Button DONE

    Auto Aim DONE

    AMP DONE

    TRAP  DONE

    UNJAM Button DONE

    //Shoot speaker:Run outake-> push note out
    //Amp scoring: (Note must be in blue wheels) -> Extend elevator-> Flip Wrist ->Outtake
    //Intake from human player: Pivot the box-> Intake with the blue wheels/feeder
    // Trap: (Note must be in blue wheels) pivot box while hanging->extend elevator + pivot wrist -> Outake
    


    
    */
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`



    new JoystickButton(driver, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));

    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox,
    //                    4).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new oInstantCommand(drivebase::lock, drivebase)));

driver_limelightButton.whileTrue(new teleoplimelight(
      limelight,
      drivebase,
      () -> driver.getLeftY(),
      () -> driver.getLeftX(), 

      () -> driver.getRightX(), () -> false));





//climbsub.shuffleboard();
projectilesub.elevatorshuffle();
intakesub.shuffleboard();
boxpivsub.encoder();





//  intakeButton.whileTrue(

// new SequentialCommandGroup(new intakesub().intakeCommand(0.5).withTimeout(0.5),new Boxpiv().pidcCommand(10)
  

// ));
//UnjamButton



UnjamButton.whileTrue(new SequentialCommandGroup(intakesub.UnjamFeeder(0.3))).whileFalse(intakesub.UnjamFeeder(0));


//AMPButton



AMPButton.whileTrue(new SequentialCommandGroup(projectilesub.Outtake(.56))).whileFalse(projectilesub.Outtake(0));

// AMPButton.whileTrue(new SequentialCommandGroup(projectilesub.elevatorcmd(-13.21431827545166),new WaitCommand(.5)));
// AMPButton.whileTrue(new SequentialCommandGroup(projectilesub.wristcmd(0),new WaitCommand(.5),projectilesub.Outtake(.3)));
// if (operator.getRawButtonPressed(4) == true){
  
//   new SequentialCommandGroup(projectilesub.Outtake(.8));  

// }

TrapButton.whileTrue(new SequentialCommandGroup(boxpivsub.boxpivcmdTO(10),projectilesub.elevatorcmd(10),new WaitCommand(0.5),projectilesub.wristcmd(10),new WaitCommand(1),projectilesub.Outtake(.3)));

// intakegroundButton
intakeButton.whileTrue(
new SequentialCommandGroup(
   intakesub.intakepid(9), new  WaitUntilCommand(0.5),intakesub.intakefeaderCommand(.50))
   ).whileFalse(intakesub.intakepid(0));
   
//intakehumanplayer
intakehmanplayerButton.whileTrue(
new SequentialCommandGroup(
  boxpivsub.boxpivcmdTO(-10),
  new ParallelCommandGroup(projectilesub.Outtake(-0.5),intakesub.intakefeaderCommand(-0.3))
  
  

)

);

//Driver uses this button th pivot the box up and drive and lock into the chain
climbPivButton.whileTrue(
  new SequentialCommandGroup(boxpivsub.boxpivcmdTO(10), new WaitCommand(3),boxpivsub.boxpivcmdTO(0))


  );

//someone else shoots out stabilizers and pulls the robot up





  // speakerButton.whileTrue(
  //   new SequentialCommandGroup(boxpivsub.boxpivcmdTO(48),new WaitCommand(1),projectilesub.Outtake(0.7))


  // );
  
    
  
    
		
    SmartDashboard.putData("Auto chooser",autoChooser);
    // Auto Aim Speaker 
    speakerButton.whileTrue(
      new SequentialCommandGroup(new ParallelDeadlineGroup(new teleoplimelight(
        limelight,
        drivebase,
        () -> driver.getLeftY(),
        () -> driver.getLeftX(), 
  
        () -> driver.getRightX(), () -> false),new LookUpShot(boxpivsub, projectilesub,() ->drivebase.calcDistToSpeaker()))
      ));
      
        
    
      
      

    


    
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }
  public void camera(){
    
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}