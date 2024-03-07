// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems.swervedrive;
 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


 
public class candle extends SubsystemBase{
    private final CANdle m_candle = new CANdle(1, "rio");
    private final int LedCount = 300;
    
 
    public candle(){}
    @Override
    public void periodic(){
 
    }
    public void setLightGreen(){
        m_candle.setLEDs(0, 255, 0);
       
    }
    public void setLightRed(){
        m_candle.setLEDs(255, 0, 0);
       
    }
    public void setLightBlue(){
        m_candle.setLEDs(0, 0, 255);
       
    }
    public Command GreenCommand(){
        return run(
 
        () -> setLightGreen()
 
        );


 
    }
        public Command RedCommand(){
        return run(
 
        () -> setLightRed()
 
        );


 
    }
        public Command BlueCommand(){
        return run(
 
        () -> setLightBlue()
 
        );


 
    }
 
}