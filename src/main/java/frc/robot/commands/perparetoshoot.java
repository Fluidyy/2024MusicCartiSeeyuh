// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;


// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.subsystems.Boxpiv;
// import frc.robot.subsystems.intakesub;
// import frc.robot.subsystems.projectiles;
// import frc.robot.subsystems.lookuptable.setpoint;
// import frc.robot.subsystems.lookuptable.setpoint.GameState;
// import frc.robot.subsystems.lookuptable.setpoint.GameState;

// import edu.wpi.first.wpilibj.Timer;

// public class perparetoshoot extends Command {

//     setpoint m_setpoints;
//     Boxpiv m_armSubsystem;
//     projectiles m_shooterSubsystem;
//     BooleanSupplier m_haveNote;
//     boolean m_isDone;
//     boolean m_runShooter;
//     intakesub s_intake;
//      Timer timeer ;
//       private boolean feederActive = false;

//     /** Constructor - Creates a new prepareToShoot. */      
//     public perparetoshoot(setpoint setpoints, BooleanSupplier haveNote, Boxpiv armSub, projectiles shootSub,intakesub intake) {
    
//         m_setpoints = setpoints;
//         s_intake = intake;
//         m_armSubsystem = armSub;
//         m_shooterSubsystem = shootSub;
//         m_haveNote = haveNote;

//         addRequirements(armSub, shootSub,s_intake);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//        timeer = new Timer();
//        timeer.reset();
//         m_isDone = false;
      

//         // If Shooter setpoints are zero, don't bother to check if it is up to speed
//         m_runShooter = (m_setpoints.shooterLeft != 0.0 || m_setpoints.shooterRight != 0.0);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {

            
//         // Set Shooter setpoints...
        // m_shooterSubsystem.setShooterSetpoints(m_setpoints);
        // m_armSubsystem.lookuptable(m_setpoints);


//         // After we have a Note in the Stage, bring Arm to requested position
//         // Don't require a Note if we are trying to STOW the arm
        
        

//         // Exit once Arm is at setpoint and Shooter setpoint is != 0 and Shooter is up to speed 
        // if (m_armSubsystem.isarmthere(m_setpoints) && (m_runShooter && m_shooterSubsystem.check(m_setpoints))) {
        //         timeer.start();
        //         if(timeer.get() <= 2){
        //             s_intake.FeederMotor1(-0.5);
        //         }
        //     }
        // }

        

        
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         // Don't turn off anything unless we have been commanded to STOWED position

       
//         }

    

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return m_isDone;
//     }
// }
