// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lookuptable.setpoint;

import frc.robot.subsystems.Boxpiv;
import frc.robot.subsystems.lookuptable.ShoterPreset;
import frc.robot.subsystems.lookuptable.lookuptable;
import frc.robot.Constants;
import frc.robot.subsystems.projectiles;


public class LookUpShotoot extends Command {

    Boxpiv m_Boxpiv;
    setpoint m_setpoints;

    ShoterPreset m_shotInfo;
    lookuptable m_lookuLookuptable;
    DoubleSupplier m_distance;
    projectiles m_shooter;

    

    /** Constructor - Creates a new prepareToShoot. */
    public LookUpShotoot(Boxpiv armSub, projectiles shooter,DoubleSupplier distance) {
        
        m_Boxpiv = armSub;
        m_shooter = shooter;
       
        m_lookuLookuptable = new lookuptable();
        m_distance = distance;
        m_setpoints = Constants.LOOKUP;


        addRequirements(armSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double distance = m_distance.getAsDouble();
        m_shotInfo = m_lookuLookuptable.getShooterPreset(distance);
        
        m_setpoints.arm = m_shotInfo.getArmAngle();
        m_setpoints.shooterLeft = m_shotInfo.getLeftShooter();
        m_setpoints.shooterRight = m_shotInfo.getRightShooter();

       

        m_Boxpiv.boxpivcmdLO(m_setpoints);

        // Bring Shooter to requested speed
        m_shooter.setoutakeLO(m_setpoints.shooterLeft, m_setpoints.shooterRight);



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the Shooter but keep the Arm control going
    
        // Don't stop the shooter in auto - copy this file for auto
    }

    // Command never ends on its own - it has to be interrupted.
    @Override
    public boolean isFinished() {
        return false;
    }
}
