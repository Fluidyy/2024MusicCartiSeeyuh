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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class underthebumper extends SubsystemBase{
    private Timer AutoTimer = new Timer();
    private Timer AutoTimer3 = new Timer();

    private CANSparkMax underbumpR = new CANSparkMax(9,MotorType.kBrushless);
    private CANSparkMax underbumpL = new CANSparkMax(10,MotorType.kBrushless);
    private CANSparkMax FeederMotor = new CANSparkMax(11,MotorType.kBrushless);
    

    private RelativeEncoder uEncoder = underbumpL.getEncoder();
    private DigitalInput beambreaker = new DigitalInput(0);
    private PIDController pid = new PIDController(0.05, 0, 0);

    public underthebumper(){}
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam1", beambreaker.get());

    }
    @Override
  public void simulationPeriodic()
  {
  } 
    
    public void FeederMotor (){
        FeederMotor.set(0.3);
        
    }
    public void FeederMotor1 (double speed){
        FeederMotor.set(speed);
        
    }
    public void FeederMotorback (double spped){
        FeederMotor.set(spped);
    }
    public void intakecons(){
        underbumpL.set(0.3);
        underbumpR.set(0.3);
    }
    public void intakeCommand(double speed){
        underbumpL.set(speed);
        underbumpR.set(speed);
    }
       public void intakeCjiommand(double speed){
        underbumpL.set(speed);
        underbumpR.set(speed);
    }
    public Command feedercmd(double speed){
        return(
            run(
                () -> FeederMotorback(speed)
            )
        );       
        
    }
    public Command intakeandfeederauto(double speed,double feaderf,double time){
    
        
        return new Command() {
            @Override
            public void initialize() {
                AutoTimer.reset();
                AutoTimer.start();
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                // Assuming setpid() calculates the speed based on PID
                underbumpL.set(-speed);
                underbumpR.set(speed);
                FeederMotor.set(feaderf);


            
            
            }
    
            @Override
            public void end(boolean interrupted) {
                underbumpL.set(0);
                underbumpR.set(0);
                FeederMotor.set(0);
                // Stop the motor when the command ends or is interrupted
            }
    
            @Override
            public boolean isFinished() {
                return AutoTimer.get() > time; // Check if the setpoint is reached
            }
        };}
        public Command intakeandfeeder(double speed,double feaderf){
    
        
        return new Command() {
            @Override
            public void initialize() {
                
                // Initialization code, such as resetting encoders or PID controllers
            }
    
            @Override
            public void execute() {
                // Assuming setpid() calculates the speed based on PID
                
                // if(beambreaker.get() == true){
                underbumpL.set(-speed);
                 underbumpR.set(speed);
                 FeederMotor.set(-feaderf);

                
                // else if(!beambreaker.get()){
                //     underbumpL.set(0);
                //     underbumpR.set(0);
                //     FeederMotor.set(0);
                // }
               


            
            
            }
    
            @Override
            public void end(boolean interrupted) {
                underbumpL.set(0);
                underbumpR.set(0);
                FeederMotor.set(0);
                // Stop the motor when the command ends or is interrupted
            }
    
            @Override
            public boolean isFinished() {
                return !beambreaker.get(); // Check if the setpoint is reached
            }
        };}


        public Command intakeconsCommand(){
        return(

        run(
            () -> intakecons()
            

        )
        );

    }
    public Command intakecmd(double speed){
        return(

        run(
            () -> intakeCommand(speed)
            

        )
        );

    }
    
   

    public double setpid(double setpoint){
        pid.setSetpoint(setpoint);
        double move = pid.calculate(uEncoder.getPosition());
        return move;
    }
    public double encoder(){
        return uEncoder.getPosition();
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




    // public Command intakepid(double setpoint){
    
        
    //     return new Command() {
    //         @Override
    //         public void initialize() {
    //             // Initialization code, such as resetting encoders or PID controllers
    //         }
    
    //         @Override
    //         public void execute() {
    //             double speed = setpid(setpoint); // Assuming setpid() calculates the speed based on PID
    //             intakepivmotor(speed);
            
            
    //         }
    
    //         @Override
    //         public void end(boolean interrupted) {
    //             setmotor(0); // Stop the motor when the command ends or is interrupted
    //         }
    
    //         @Override
    //         public boolean isFinished() {
    //             return pivencoder.getPosition() >= setpoint-1 && pivencoder.getPosition()<= setpoint+1; // Check if the setpoint is reached
    //         }
    //     };
    // }


     

}