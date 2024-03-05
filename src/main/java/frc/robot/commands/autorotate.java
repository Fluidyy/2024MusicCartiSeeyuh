package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.photo.Photo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.PhotonVision;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;


import static edu.wpi.first.units.Units.*;



public class autorotate extends Command {
    private PhotonVision s_vision;
    private SwerveSubsystem s_Swerve; 

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double prevdistance = 100;
    private double setpoint =  15;

   public autorotate(PhotonVision photonvision ,SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        s_vision = photonvision;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
 
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
    

    //     // vision.refreshValues();
    //     // double distance = s_limelight.calculateDistance(s_limelight.getRY(), 10, 0, 30);
    //     // if (distance < prevdistance){
    //     //     prevdistance = distance;
    //     //     arm.pid(setpoint*1.1);
    //     // }
    //     // if (distance > prevdistance){
    //     //     prevdistance = distance;
    //     //     arm.pid(setpoint/1.1);

        
    //    /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND);
        // double distFactor = 0.2 * (s_vision.getRZ() > 5 ? 5 : s_vision.getRZ());
    //     /* Get rotation */
        PIDController turnController = new PIDController(0.01, 0.0001, 0.000005);
        turnController.enableContinuousInput(-180, 180);
        double  rotationSpeed = -turnController.calculate(s_vision.getLatestResult().getBestTarget().getYaw(), 0);
    // 2    /* Drive */
    //     // s_Swerve.driveCommand(
    //     //     new Translation2d(translationVal, strafeVal).times(s_Swerve.maximumSpeed), 
    //     //     -rotate,/

    //     /*\7\,..0.0.0. */
    //     //     !robotCentricSup.getAsBoolean(), 
    //     //     true
    //     // );
        s_Swerve.drive1(
            new Translation2d(translationVal,strafeVal).times(s_Swerve.maximumSpeed),
            -rotationSpeed * 0.1,
            robotCentricSup.getAsBoolean()
            
        );
    //     ;
        // s_Swerve.driveCommand(translationSup, translationSup, rotationSup);
    }
 
}
