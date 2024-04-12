// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.autointake;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


// import frc.robot.subsystems.intakesub;
import java.io.File;
import java.util.logging.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

// import frc.robot.subsystems.Limelight;
// // import frc.robot.subsystems.PhotonVision;
// import frc.robot.subsystems.PhotonVision;


// // import frc.robot.subsystems.PhotonVision;

// import frc.robot.subsystems.limek1;
// import frc.robot.subsystems.lookuptable.lookuptable;
import frc.robot.commands.LookUpShotoot;
import frc.robot.commands.autorotate;
import frc.robot.subsystems.Boxpiv;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.cam2photon;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.projectiles;
// import frc.robot.subsystems.outake;
import frc.robot.subsystems.underthebumper;
// import frc.robot.subsystems.PhotonVision;

import frc.robot.subsystems.PhotonVision;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  //  private lookuptable m_Visionlookuptable = new lookuptable();


  // The robot's subsystems and c ommands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // private final Limelight limelight = new Limelight();



  private final Boxpiv boxpivsub = new Boxpiv();
 private final Climb climbsub = new Climb();
  private final projectiles projectilesub = new projectiles();
  private final PhotonVision lime = new PhotonVision(drivebase);
  private final cam2photon cam2 = new cam2photon();
  private final underthebumper newintake = new underthebumper();


  
  // private final PhotonVision Vision = new PhotonVision(drivebase);


                                       
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  Joystick operator = new Joystick(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();




  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driver = new XboxController(0);

  private final JoystickButton driver_limelightButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);


  private final JoystickButton climbpurd = new JoystickButton(driver, XboxController.Button.kRightStick.value);
  private final JoystickButton subwoofer = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton podium = new JoystickButton(operator, XboxController.Button.kA.value);

  //private final JoystickButton intakehmanplayerButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
   private final JoystickButton AMPButton = new JoystickButton(operator, XboxController.Button.kY.value);
      private final JoystickButton Outake = new JoystickButton(operator, XboxController.Button.kBack.value);
      private final JoystickButton autointake = new JoystickButton(driver, XboxController.Button.kX.value);
            private final JoystickButton autoroate = new JoystickButton(driver, XboxController.Button.kX.value);
      
  


  private final POVButton UnjamButton= new POVButton(operator, 180);
    private final POVButton feaderf= new POVButton(operator, 0);
      private final POVButton outakeunjam= new POVButton(operator, 90);
    private final POVButton outakeunjam1= new POVButton(operator, 270);
  private final JoystickButton climbr = new JoystickButton(driver, XboxController.Button.kBack.value);
   private final JoystickButton climbpul = new JoystickButton(driver, XboxController.Button.kStart.value);//p4
    private final JoystickButton climbpur = new JoystickButton(driver, XboxController.Button.kLeftStick.value);//p1

  











  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()



  {
    // hub.setSwitchableChannel(true);
    // m_vision.useLimelight(true);
    // m_vision.setAlliance(Alliance.Blue);
    // m_vision.trustLL(true);
     // Correct pose estimate with vision measurements
     Rotation2d cRotation2d = drivebase.getPose().getRotation();

    

      

    
    boxpivsub.encoder();
    // projectilesub.shuffleboard();
    // drivebase.llreset();





    NamedCommands.registerCommand("runintake", newintake.intakeandfeeder(0.7, 0.7));
    NamedCommands.registerCommand("pullback", newintake.intakeandfeederauto(0,0.4 ,.4));
        NamedCommands.registerCommand("pullbacko", projectilesub.Outtake(-1));
        

    NamedCommands.registerCommand("runintake2", newintake.intakeandfeederauto(0.8,-0.80,2));
    NamedCommands.registerCommand("boxpivclose",boxpivsub.boxpivcmdAU(-8.04736328125));
    NamedCommands.registerCommand("boxpivfar",boxpivsub.boxpivcmdAU1(-6.8));
    NamedCommands.registerCommand("shoot",projectilesub.OtakeAU(.60));

    
    //Make Multiple of the below shoot speakers because you need to manually find the setpoints and speed for each angle
    
    //autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("hi", new SequentialCommandGroup());
   
        autoChooser.addOption("2 piece stage ", new PathPlannerAuto("2 Piece Stage"));
            autoChooser.addOption("2 piece amp", new PathPlannerAuto("Ashrith"));
             autoChooser.addOption("2 piece center", new PathPlannerAuto("Copy of 2 Piece Center Real hi"));


    drivebase.odometrygetshuffleboard();

    configureBindings();
     SmartDashboard.putData("Auto chooser",autoChooser);
   
    // projectilesub.shuffleboard();
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

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driver.getRawAxis(2));

    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    teleop closedFieldRelOperator = new teleop(
      drivebase,
      () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),

      () -> driver.getRightX(), () -> true);
      

    drivebase.setDefaultCommand(closedFieldRelOperator);
    newintake.intakeconsCommand();
    




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
  



// driver_limelightButton.whileTrue(new autorotate(
//       lime,
//       drivebase,
//       () -> driver.getLeftY(),
//       () -> driver.getLeftX(), 
//       () -> driver.getRightX(), () -> false));





//shuffleboards












UnjamButton.whileTrue(new SequentialCommandGroup(newintake.feedercmd(-0.7))).whileFalse(newintake.feedercmd(0));


feaderf.whileTrue(new ParallelCommandGroup(newintake.feedercmd(0.7),projectilesub.Outtake(-1))).whileFalse(new ParallelCommandGroup(newintake.feedercmd(0),projectilesub.Outtake(0)));


subwoofer.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-3.43359375),projectilesub.Outtake(0.65))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));
podium.whileTrue(new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-8),projectilesub.Outtake(0.65))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));
 
AMPButton.whileTrue(new SequentialCommandGroup(boxpivsub.boxpivcmdTOamp(-19.45),new ParallelCommandGroup(projectilesub.Outtake(0.20),boxpivsub.boxpivcmdTO(-19.45)))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTO(0)));
// AMPButton.whileTrue(new SequentialCommandGroup(new ParallelDeadlineGroup(boxpivsub.boxpivcmdTOamp(-13), projectilesub.OuttakeAmp(.20)), new ParallelDeadlineGroup(boxpivsub.boxpivcmdTOamp(-20.1),newintake.autoampshit(-0.8), projectilesub.OuttakeAmp(.19)))).whileFalse(new ParallelCommandGroup(projectilesub.Outtake(0),boxpivsub.boxpivcmdTOamp(0),newintake.intakeandfeederauto(0,0,0)));

Outake.whileTrue(projectilesub.Outtake(.8)).whileFalse(projectilesub.Outtake(0));


// intakegroundButton
intakeButton.whileTrue(
(
   newintake.intakeandfeeder(0.65, 0.75)))
   .whileFalse(newintake.intakeandfeeder(0,0));
   
// wrist.whileTrue(
//   projectilesub.wristcmd(17.857131958007812)
// ).whileFalse(projectilesub.wristcmd(17.857131958007812));

//Driver uses this button th pivot the box up and drive and lock into the chain
// climbPivButton.whileTrue(
//   new ParallelCommandGroup(boxpivsub.boxpivcmdTO(-8.61962890625))


//   ).whileFalse(boxpivsub.boxpivcmdTO(-11));
// climbButton.whileTrue(
//   new SequentialCommandGroup(climbsub.ClimbCmd1(-0.75))
// ).whileFalse(climbsub.ClimbCmd1(0));

// climbButton.whileTrue(
//   new SequentialCommandGroup(climbsub.ClimbCmd2(-0.75))
// ).whileFalse(climbsub.ClimbCmd2(0));






outakeunjam1.whileTrue(projectilesub.Outtake(-0.5)).whileFalse(projectilesub.Outtake(0));




autointake.whileTrue(new autointake(
      cam2,
      drivebase,
      () -> driver.getLeftY(),
      () -> driver.getLeftX(), 

      () -> driver.getRightX(), () -> false));  

  
  
		
   autoroate.whileTrue(new autorotate(lime,drivebase,() -> driver.getLeftY(),
      () -> driver.getLeftX(), 

      () -> driver.getRightX(), () -> false));  

    // Auto Aim Speaker 
    driver_limelightButton.whileTrue(
      
    new LookUpShotoot(boxpivsub, projectilesub,() ->drivebase.calcDistToSpeaker(), newintake))
      ;
      
        
      
      
    climbr.toggleOnTrue(climbsub.solenoidCommand(true,-0.6)).toggleOnFalse(climbsub.solenoidCommand(false,0));
    climbpul.whileTrue(climbsub.ClimbCmd1(0.3)).whileFalse(climbsub.ClimbCmd1(0));
    climbpur.whileTrue(climbsub.ClimbCmd2(0.3)).whileFalse(climbsub.ClimbCmd2(0));
    
    climbpurd.whileTrue(climbsub.climbcmd(0.6)).whileFalse(climbsub.climbcmd(0));

    
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
    // return new SequentialCommandGroup();
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