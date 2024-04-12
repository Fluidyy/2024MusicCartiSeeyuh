package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import frc.robot.subsystems.lookuptable.setpoint;
import frc.robot.Constants;
import frc.robot.subsystems.lookuptable.setpoint;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.*;


public class Boxpiv extends SubsystemBase{
    private TalonFX boxpivR = new TalonFX(6);
    private TalonFX boxpivL = new TalonFX(7);
    private double m_armSetpoint;

    private double m_tolerance;

    
      
    private PIDController pid = new PIDController(0.05, 0, 0);
        private TrapezoidProfile.State m_tpState = new TrapezoidProfile.State(0.0, 0.0);
        private Timer AutoTimer = new Timer();
    

    public Boxpiv(){
        

    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Boxpiv", boxpivR.getPosition().getValueAsDouble());  

    }
    public void boxpivotMotor(double speed){
        boxpivR.set(speed);
        boxpivL.set(-speed);
       
       
    }


    public void encoder(){
       
    
    }

    public double setSetpoint(double setpoint){
        
        pid.setSetpoint(setpoint);
        double move = pid.calculate(boxpivR.getPosition().getValueAsDouble());
        return move;
        
       
        

    }

    // public Command pidcCommand(double setpoint){
    
        
    //     return runEnd(
    //         () -> {
    //             double speed = setpid(setpoint);
    //             boxpivotMotor(speed);

    //         },
    //         () -> {
    //            double speed1 = setpid(0);
    //            boxpivotMotor(speed1);

                
    //         });


    // }
    public void lookuptable(setpoint setpoint){

        double speed = setSetpoint(setpoint.arm);
        boxpivotMotor(speed);

    }
    // public boolean isarmthere(setpoint setpoint){
    //    return boxpivR.getPosition().getValueAsDouble() >= setpoint.arm-1 && boxpivR.getPosition().getValueAsDouble()<= setpoint.arm+1;
    // }
    public Command boxpivcmdLO(setpoint setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                double speed = setSetpoint(setpoint.arm); // Assuming setpid() calculates the speed based on PID
                boxpivotMotor(speed);
            }
    
            @Override
            public void end(boolean interrupted) {
                boxpivotMotor(0); // Stop the motor when the command ends or is interrupted
            }
    
            @Override
            public boolean isFinished() {
                return boxpivR.getPosition().getValueAsDouble() == setpoint.arm ; // Check if the setpoint is reached
            }
        };
    }
//Teleop Box Pivot. Used for climb trap amp
    public Command boxpivcmdTO(double setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                double speed = setSetpoint(setpoint); // Assuming setpid() calculates the speed based on PID
                boxpivotMotor(speed);
            }
    
            @Override
            public void end(boolean interrupted) {
                boxpivotMotor(0); // Stop the motor when the command ends or is interrupted
                //setSetpoint(0);
                

            }
            
    
            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }    

    public Command boxpivcmdAU(double setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                double speed = setSetpoint(setpoint); // Assuming setpid() calculates the speed based on PID
                boxpivotMotor(speed);
            }
    
            @Override
            public void end(boolean interrupted) {
                boxpivotMotor(0);
                 // Stop the motor when the command ends or is interrupted
                //setSetpoint(0);
                

            }
            
    
            @Override
            public boolean isFinished() {
            return false;
                        }
        };
    }    

    public Command boxpivcmdAU1(double setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                AutoTimer.reset();
                
            }
    
            @Override
            public void execute() {
                double speed = setSetpoint(setpoint); // Assuming setpid() calculates the speed based on PID
                boxpivotMotor(speed);
            }
    
            @Override
            public void end(boolean interrupted) {
                boxpivotMotor(0); // Stop the motor when the command ends or is interrupted
                //setSetpoint(0);
                

            }
            
    
            @Override
            public boolean isFinished() {
            return false;
                        }
        };
    }    

     public Command boxpivcmdTOamp(double setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
                pid.setP(0.06);
            }
    
            @Override
            public void execute() {
                double speed = setSetpoint(setpoint); // Assuming setpid() calculates the speed based on PID
                boxpivotMotor(speed);
            }
    
            @Override
            public void end(boolean interrupted) {
                boxpivotMotor(0); // Stop the motor when the command ends or is interrupted
                //setSetpoint(0);
                

            }
    
            @Override
            public boolean isFinished() {
                return boxpivR.getPosition().getValueAsDouble() >= setpoint-0.5 && boxpivR.getPosition().getValueAsDouble()<= setpoint+0.5;
            }
        };
    }  
      

     public Command boxpivcmdTOampslow(double setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                pid.setP(0.01);
                double speed = setSetpoint(setpoint); // Assuming setpid() calculates the speed based on PID
                boxpivotMotor(speed);
            }
    
            @Override
            public void end(boolean interrupted) {
                boxpivotMotor(0); // Stop the motor when the command ends or is interrupted
                //setSetpoint(0);
                

            }
    
            @Override
            public boolean isFinished() {
                return boxpivR.getPosition().getValueAsDouble() >= setpoint-1 && boxpivR.getPosition().getValueAsDouble()<= setpoint+1;
            }
        };
    }    

    public Command speed(double speed){

        return run(
            

        () -> boxpivotMotor(speed)
        );
    }


    }


    