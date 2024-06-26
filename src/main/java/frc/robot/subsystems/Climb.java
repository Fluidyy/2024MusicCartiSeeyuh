package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;


public class Climb extends SubsystemBase{


    private static final int PH_CAN_ID1 = 20;
    private static final int SOLENOID_CHANNEL1 = 1;
    private static final int SOLENOID_CHANNEL2 = 2;
    PneumaticHub m_pH1 = new PneumaticHub(PH_CAN_ID1);
    Solenoid m_solenoid1 = m_pH1.makeSolenoid(0);

    private CANSparkMax ClimbMotor1 = new CANSparkMax(14, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);;
    private CANSparkMax ClimbMotor2 = new CANSparkMax(15, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder ClimbEncoder = ClimbMotor1.getEncoder();




    public Climb(){
        
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climb", ClimbEncoder.getPosition());

    }
    public void shuffleboard(){
        
    }
    public void setSolenoid(boolean hi,double speed){

        m_solenoid1.set(hi);
        ClimbMotor1.set(speed);
        ClimbMotor2.set(-speed);

      


    }
    //    public void setSolenoidfalse(){
    //     m_solenoid1.set(false);
    //     m_solenoid2.set(true);


    // }

    public void runClimbMotor(double speed){
        ClimbMotor1.set(speed);
        ClimbMotor2.set(-speed);

    }

    public boolean climbSPChecker(){
        return ClimbEncoder.getPosition() == 5;
        
    }



    // public Command ClimbSPchecker(){
    //     return run(

    //     () -> climbSPChecker()
            



    //     );

    // }//
    //Pulls solenoid down
    public Command solenoidCommand(boolean hi,double speed){
        return run(

        () -> setSolenoid(hi,speed)
            


    
        );
    }



    // }
    //This is for pulling up

    public Command climbcmd(double speed){
        return run(

        () -> runClimbMotor(speed)
            



        );



    }    
    public Command ClimbCmd1(double speed){
        return run(

        () -> ClimbMotor1.set(speed)
            



        );



    }    

    public Command ClimbCmd2(double speed){
        return run(

        () -> ClimbMotor2.set(-speed)
            



        );



    }    


    

    // public Command ClimbCmd1(){
    
        
    //     return new Command() {
    //         @Override
    //         public void initialize() {
    //             ClimbEncoder.setPosition(0);
                
    //         }
    
    //         @Override
    //         public void execute() {
    //             runClimbMotor();
    //         }
    
    //         @Override
    //         public void end(boolean interrupted) {
    //              // Stop the motor when the command ends or is interrupted
    //             //setSetpoint(0);
                

    //         }
    
    //         @Override
    //         public boolean isFinished() {
    //             return ClimbEncoder.getPosition() == 5; // Check if the setpoint is reached
    //         }
    //     };
    // }    



}
