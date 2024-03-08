package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class intakesub extends SubsystemBase{
    private CANSparkMax intakemotorR = new CANSparkMax(16,MotorType.kBrushless);
    private CANSparkMax intakepidmotor = new CANSparkMax(17,MotorType.kBrushless);
    private CANSparkMax FeederMotor = new CANSparkMax(21,MotorType.kBrushless);

    private RelativeEncoder pivencoder = intakepidmotor.getEncoder();
    private PIDController pid = new PIDController(0.05, 0, 0);

    public intakesub(){}
    @Override
    public void periodic(){

    }
    public void setmotor(double speed){
       
        intakemotorR.set(speed);
        
    }public void setmotorf(double speed){
       
        intakemotorR.set(speed);
        FeederMotor.set(speed);
    }
    public void setmotorstop(double speed){
       
        intakemotorR.set(speed);
        FeederMotor.set(speed);
        
    }
     public void setmotorfeeder(double speed){
       
        intakemotorR.set(speed);
        FeederMotor.set(speed*0.7);
    }
    public void FeederMotor (){
        FeederMotor.set(0.3);
        
    }
    public void FeederMotorback (double spped){
        FeederMotor.set(spped);
    }
    public void intakepivmotor(double speed){
        intakepidmotor.set(speed);
        
    }
   

    public double setpid(double setpoint){
        pid.setSetpoint(setpoint);
        double move = pid.calculate(pivencoder.getPosition());
        return move;
    }
    public double encoder(){
        return pivencoder.getPosition();
    }
    public Command feederCommand(){

        return run(
            

        () -> FeederMotor()
        );
    }
    public Command feederbackCommand(){

        return run(
            

        () -> FeederMotorback(-0.3)
        );
    }

    // public Command pidcCommand(double setpoint){
    
        
    //     return runEnd(
    //         () -> {
    //             double speed = setpid(setpoint);
    //             setmotor(speed);

    //         },
    //         () -> {
    //            double speed1 = setpid(0);
    //            setmotor(speed1);

                
    //         });


    // }


    public Command intakeCommand(double speed){
        return run(

        () -> setmotor(speed)
        

        );
    }

    public Command intakeCommandf(double speed){
        return run(

        () -> setmotorf(speed)
        

        );
    }
    public Command intakeCommandfeeder(double speed){
        return run(

        () -> setmotorfeeder(speed)

        

        );
    }


    public Command intakepid(double setpoint){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                double speed = setpid(setpoint); // Assuming setpid() calculates the speed based on PID
                intakepivmotor(speed);
            
            
            }
    
            @Override
            public void end(boolean interrupted) {
                setmotor(0); // Stop the motor when the command ends or is interrupted
            }
    
            @Override
            public boolean isFinished() {
                return pivencoder.getPosition() >= setpoint-1 && pivencoder.getPosition()<= setpoint+1; // Check if the setpoint is reached
            }
        };
    }


      public Command intakepidandfeeder(double setpoint,double speed){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                double move = setpid(setpoint); // Assuming setpid() calculates the speed based on PID
                intakepivmotor(move);
                setmotorfeeder(speed);
            
            
            }
    
            @Override
            public void end(boolean interrupted) {
                setmotor(0); // Stop the motor when the command ends or is interrupted
            }
    
            @Override
            public boolean isFinished() {
                return false; // Check if the setpoint is reached
            }
        };
    }


     public Command intakefeaderCommand(double speed){
    
        
        return new Command() {
            @Override
            public void initialize() {
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                // Assuming setpid() calculates the speed based on PID
                setmotorfeeder(speed);

            
            
            }
    
            @Override
            public void end(boolean interrupted) {
                setmotorstop(0);
                // Stop the motor when the command ends or is interrupted
            }
    
            @Override
            public boolean isFinished() {
                return false; // Check if the setpoint is reached
            }
        };
    }
public Command UnjamFeeder(double spped){
        return run(

        () -> FeederMotorback(spped)
        

        );
    }

    
public void shuffleboard(){
    SmartDashboard.putNumber("intake position", pivencoder.getPosition());
}

}