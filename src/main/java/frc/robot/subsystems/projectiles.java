package frc.robot.subsystems;
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
import frc.robot.subsystems.lookuptable.setpoint;
import frc.robot.Constants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;


public class projectiles extends SubsystemBase{
    private CANSparkFlex OutakeR = new CANSparkFlex(26, MotorType.kBrushless);
    private CANSparkFlex OutakeL = new CANSparkFlex(25, MotorType.kBrushless);
    private CANSparkMax wrist = new CANSparkMax(19, MotorType.kBrushless);
    private CANSparkMax elevator = new CANSparkMax(21, MotorType.kBrushless);
    private PIDController wristpiPidController = new PIDController(0.01, 0, 0);
    private PIDController elevatorpidcController = new PIDController(0.01, 0, 0);
    private RelativeEncoder wristE = wrist.getEncoder();
    private RelativeEncoder elevatorE = elevator.getEncoder();
     private RelativeEncoder OutakeE = OutakeL.getEncoder();


    

public projectiles(){}
@Override
public void periodic(){

}

public void setoutakeLO(double leftv, double rightv){
    OutakeL.set(leftv);
    OutakeR.set(rightv);
    
}
public void shuffleboard(){
    SmartDashboard.putNumber("wrist", wristE.getPosition());
   
    SmartDashboard.putNumber("elevator",elevatorE.getPosition());

    SmartDashboard.putNumber("Outake Velocity", OutakeE.getVelocity());
}

public void setoutakeTE(double speed){
    OutakeL.set(speed);
    OutakeR.set(speed);
    
}

public void Unjam(){
    OutakeL.set(-.3);
    OutakeR.set(-.3);
    

}

public double elevatorpid(double setpoint){
    elevatorpidcController.setSetpoint(setpoint);
    return elevatorpidcController.calculate(elevatorE.getPosition());

}

public double wristpid(double setpoint){
     wristpiPidController.setSetpoint(setpoint);
    return wristpiPidController.calculate(wristE.getPosition());
}
public Command elevatorcmd(double setpoint){
    
        
    return new Command() {
        @Override
        public void initialize() {
            // Initialization code, such as resetting encoders or PID controllers
        }

        @Override
        public void execute() {
            double speed = elevatorpid(setpoint); // Assuming setpid() calculates the speed based on PID
            elevator.set(speed);


        }

        @Override
        public void end(boolean interrupted) {
            elevator.set(0);; // Stop the motor when the command ends or is interrupted
        }

        @Override
        public boolean isFinished() {
            return elevatorE.getPosition() == setpoint ; // Check if the setpoint is reached
        }
    };
}
public Command wristcmd(double setpoint){
    
        
    return new Command() {
        @Override
        public void initialize() {
            // Initialization code, such as resetting encoders or PID controllers
        }

        @Override
        public void execute() {
            double speed = wristpid(setpoint); // Assuming setpid() calculates the speed based on PID
            wrist.set(speed);


        }

        @Override
        public void end(boolean interrupted) {
            wrist.set(0); // Stop the motor when the command ends or is interrupted
        }

        @Override
        public boolean isFinished() {
            return wristE.getPosition() == setpoint ; // Check if the setpoint is reached
        }
    };
}
public Command lookupCommand(double leftv, double rightv){
    
        
    return new Command() {
        @Override
        public void initialize() {
            // Initialization code, such as resetting encoders or PID controllers
        }

        @Override
        public void execute() {
            OutakeL.set(leftv);
            OutakeR.set(rightv);


        }

        @Override
        public void end(boolean interrupted) {
            wrist.set(0); // Stop the motor when the command ends or is interrupted
        }

        @Override
        public boolean isFinished() {
            return OutakeE.getVelocity() == leftv; // Check if the setpoint is reached
        }
    };
}

//Used 
public Command Outtake(double speed){

        return run(
            

        () -> setoutakeTE(speed)
        );
    }
public Command ProjectilesUnjam(double speed){

        return run(
            

        () -> Unjam()
        );
    }    
    
}


