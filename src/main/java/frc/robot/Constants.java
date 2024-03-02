// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.lookuptable.setpoint.GameState;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import frc.robot.subsystems.lookuptable.setpoint;
import edu.wpi.first.math.Matrix;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
      //  public static final setpoint STOWED = new setpoint(1.0, 0.4, 0.0, 0.0, GameState.STOWED);
      //   public static final setpoint INTAKE = new setpoint(3.0, 2.0, 0.0, 0.0, GameState.INTAKE);
      //   public static final setpoint SUBWOOFER = new setpoint(1.0, 1.0, 30.0,25.0,  GameState.SUBWOOFER);
      //   public static final setpoint AMP = new setpoint(88.0, 0.4, 30.0,30.0,  GameState.AMP);
      //   public static final setpoint PODIUM = new setpoint(23.0, 0.4, 50.0,50.0,  GameState.PODIUM);
      //   public static final setpoint WING = new setpoint(30.0, 0.4, 70.0,60.0,  GameState.WING);
      //   public static final setpoint PREPCLIMB = new setpoint(0.0, 0.4, 0.0,0.0,  GameState.PREPCLIMB);
      //   public static final setpoint CLIMB = new setpoint(0.0, 0.4, 0.0,0.0,  GameState.CLIMB);
      //   public static final setpoint TRAP = new setpoint(0.0, 0.4, 20.0,20.0,  GameState.TRAP);
        public static final setpoint LOOKUP = new setpoint(0.0, 0.4, 20.0,25.0,  GameState.LOOKUP);
        
        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));


  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(29)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Vision {
        public static final String kCameraName = "front_cam";
        //public static final String kCameraName = "USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(.174,-0.186, 0.588), new Rotation3d(0, Math.toRadians(10),0));

        //public static final Transform3d kRobotToCam =
                //new Transform3d(new Translation3d(0.264922,0.2465578, 0.2182876), new Rotation3d(0, Math.toRadians(22.09),Math.toRadians(5)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

}
