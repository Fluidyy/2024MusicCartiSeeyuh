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
import frc.robot.subsystems.swervedrive.candle;

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
  private final candle candle = new candle();


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

  private final JoystickButton driver_limelightButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

  private final JoystickButton climbButton = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
  private final JoystickButton climbPivButton = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
  private final JoystickButton subwoofer = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton podium = new JoystickButton(operator, XboxController.Button.kA.value);

  //private final JoystickButton intakehmanplayerButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton AutoaimSpeaker = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
   private final JoystickButton AMPButton = new JoystickButton(operator, XboxController.Button.kY.value);
 
  


  private final POVButton UnjamButton= new POVButton(operator, 270);
    private final POVButton feaderf= new POVButton(operator, 90);
  private final JoystickButton driverunjam= new JoystickButton(driver, XboxController.Button.kBack.value);











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

    

      

    intakesub.shuffleboard();
    boxpivsub.encoder();
    projectilesub.shuffleboard();

    NamedCommands.registerCommand("intakedown", intakesub.intakepid(10));
    NamedCommands.registerCommand("runintake", intakesub.intakeCommand(.5));
    NamedCommands.registerCommand("TwoShootPiv",boxpivsub.boxpivcmdTO(10));
    NamedCommands.registerCommand("TwoShootS",projectilesub.Outtake(.5));
    NamedCommands.registerCommand("Feeder",intakesub.UnjamFeeder(-.5));
    NamedCommands.registerCommand("intakestop", intakesub.intakeCommand(0));
    NamedCommands.registerCommand("shootstop",projectilesub.Outtake(0));
    NamedCommands.registerCommand("Feederstop",intakesub.UnjamFeeder(0));
    
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
    


  


    new JoystickButton(driver, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));



driver_limelightButton.whileTrue(new teleoplimelight(
      limelight,
      drivebase,
      () -> driver.getLeftY(),
      () -> driver.getLeftX(), 

      () -> driver.getRightX(), () -> false));





//shuffleboards












UnjamButton.whileTrue(new SequentialCommandGroup(intakesub.UnjamFeeder(0.3))).whileFalse(intakesub.UnjamFeeder(0));


feaderf.whileTrue(intakesub.UnjamFeeder(-0.3)).whileFalse(intakesub.UnjamFeeder(0));
driverunjam.whileTrue(intakesub.UnjamFeeder(-0.3)).whileFalse(intakesub.UnjamFeeder(0));

subwoofer.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-10),projectilesub.Outtake(-1))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));
podium.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(5),projectilesub.Outtake(-1))).whileFalse(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(0)) );

AMPButton.whileTrue(new SequentialCommandGroup(boxpivsub.boxpivcmdTO(-4.0121), projectilesub.Outtake(.56))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));




// intakegroundButton
intakeButton.whileTrue(
new ParallelCommandGroup(
   intakesub.intakepidandfeeder(-17.999954223632812,-0.7))
   ).whileFalse(intakesub.intakepidandfeeder(0,0));
   


//Driver uses this button th pivot the box up and drive and lock into the chain
climbPivButton.whileTrue(
  new ParallelCommandGroup(boxpivsub.boxpivcmdTO(10), intakesub.intakepid(4))


  );
climbButton.whileTrue(
  new SequentialCommandGroup(climbsub.ClimbCmd(.5))
);









  
    
  
    
		
    SmartDashboard.putData("Auto chooser",autoChooser);
    // Auto Aim Speaker 
    AutoaimSpeaker.whileTrue(
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