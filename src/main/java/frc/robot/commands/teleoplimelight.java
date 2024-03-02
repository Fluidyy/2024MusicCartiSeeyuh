package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.Limelight;


public class teleoplimelight extends Command {
    private Limelight s_limelight;
    private SwerveSubsystem s_Swerve; 

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double prevdistance = 100;
    private double setpoint =  15;

   public teleoplimelight(Limelight limelight ,SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        s_limelight = limelight;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
 
        addRequirements(s_limelight, s_Swerve);
    }

    @Override
    public void execute() {
        
        s_limelight.refreshValues();
        // double distance = s_limelight.calculateDistance(s_limelight.getRY(), 10, 0, 30);
        // if (distance < prevdistance){
        //     prevdistance = distance;
        //     arm.pid(setpoint*1.1);
        // }
        // if (distance > prevdistance){
        //     prevdistance = distance;
        //     arm.pid(setpoint/1.1);
        
       /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND);
        double distFactor = 0.2 * (s_limelight.getRZ() > 5 ? 5 : s_limelight.getRZ());
        /* Get rotation */
        PIDController rotController = new PIDController((1.0-(0.75*distFactor))*0.2, 0.0001, 0.000005);
        rotController.enableContinuousInput(-180, 180);
        double rotate = rotController.calculate(s_Swerve.getYaw(), s_Swerve.getHeading().getDegrees() + 15*s_limelight.getRX());
        /* Drive */
        // s_Swerve.driveCommand(
        //     new Translation2d(translationVal, strafeVal).times(s_Swerve.maximumSpeed), 
        //     -rotate,/

        /*\7\,..0.0.0. */
        //     !robotCentricSup.getAsBoolean(), 
        //     true
        // );
        s_Swerve.drive1(
            new Translation2d(translationVal,strafeVal).times(s_Swerve.maximumSpeed),
            -rotate * 0.1,
            robotCentricSup.getAsBoolean()
            
        )
        ;
        // s_Swerve.driveCommand(translationSup, translationSup, rotationSup);
    }
 
}
