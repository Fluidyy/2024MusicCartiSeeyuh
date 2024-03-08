// package frc.robot.subsystems;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.*;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.RobotMap.PoseConfig;

// import frc.robot.core.MAXSwerve.MaxSwerveConstants;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// /** Reports our expected, desired, and actual poses to dashboards */
// public class PoseEstimator extends SubsystemBase {
//   private static PoseEstimator instance;

//   public static PoseEstimator getInstance() {
//     if (instance == null) instance = new PoseEstimator();
//     return instance;
//   }

//   // PoseConfig config;
//   private PoseTelemetry telemetry;
//   private Pose2d odometryPose = new Pose2d();
//   private Pose2d desiredPose = new Pose2d();
//   private Pose2d estimatePose = new Pose2d();

//   private final SwerveDrivePoseEstimator poseEstimator;
//   private final SwerveSubsystem drivetrain;

//   private PoseEstimator() {
//     // config = new PoseConfig();
//     telemetry = new PoseTelemetry(this);

//     drivetrain = Drivetrain.getInstance();

//     // Maxswerve Version from MAXSwerve.java in core
//     poseEstimator =
//         new SwerveDrivePoseEstimator(
//             MaxSwerveConstants.kDriveKinematics,
//             drivetrain.getYaw(),
//             drivetrain.getModulePositions(),
//             drivetrain.getPose(),
//             createStateStdDevs(
//                 PoseConfig.kPositionStdDevX,
//                 PoseConfig.kPositionStdDevY,
//                 PoseConfig.kPositionStdDevTheta),
//             createVisionMeasurementStdDevs(
//                 PoseConfig.kVisionStdDevX,
//                 PoseConfig.kVisionStdDevY,
//                 PoseConfig.kVisionStdDevTheta));
//   }

//   @Override
//   public void periodic() {
//     updateOdometryEstimate(); // Updates using wheel encoder data only
//     // Updates using the vision estimate
//     estimatePose = Vision.getInstance().visionBotPose();
//     if (VisionConfig.IS_LIMELIGHT_MODE && estimatePose != null) { // Limelight mode
//       double currentTimestamp =
//           Vision.getInstance().getTimestampSeconds(Vision.getInstance().getTotalLatency());
//       if (isEstimateReady(estimatePose)) { // Does making so many bot pose variables impact accuracy?
//         addVisionMeasurement(estimatePose, currentTimestamp);
//       }
//     }
//     // TODO Photonvision mode - Needs editing and filtering
//     if (VisionConfig.IS_PHOTON_VISION_MODE && estimatePose != null) { // Limelight mode
//       double photonTimestamp = Vision.getInstance().getPhotonTimestamp();
//       if (isEstimateReady(estimatePose)) { // Does making so many bot pose variables impact accuracy?
//         addVisionMeasurement(estimatePose, photonTimestamp);
//       }
//     }

//     //UNTESTED - ALWAYS SETS DRIVETRAIN ODOMETRY TO THE POSE-ESTIMATOR ODOMETRY
//     //NOT GREAT FOR ERROR CHECKING POSE ESTIMATOR! - SET TO FALSE
//     if(VisionConfig.VISION_OVERRIDE_ENABLED){
//       drivetrain.resetOdometry(getPosition());
//     }

//     // Update for telemetry
//     setEstimatedPose(getPosition());
//     setOdometryPose(drivetrain.getPose());

//     // telemetry.updatePoseOnField("VisionPose", Robot.vision.botPose);
//     telemetry.updatePoseOnField("OdometryPose", odometryPose);
//     telemetry.updatePoseOnField(
//         "EstimatedPose", estimatePose); // Need to uncomment and fix to work here.
//   }

//   // /**
//   //  * @return true if estimated pose is on the chargestation by using the field-space
//   // chargestation
//   //  *     dimensions
//   //  */
//   // public boolean isOnChargeStation() {
//   //     return ((getBestPose().getX() > 2.9 && getBestPose().getX() < 4.8)
//   //             && (getBestPose().getY() > 1.54 && getBestPose().getY() < 3.99));
//   // }

//   // /**
//   //  * Returns the most accurate pose. If we are not confident that vision is accurate,
//   //  * estimatedPose is considered to be most accurate.
//   //  *
//   //  * @return vision pose or estimated pose
//   //  */
//   // public Pose2d getBestPose() {
//   //     if (Robot.vision.visionAccurate()) {
//   //         return Robot.vision.botPose;
//   //     } else {
//   //         return estimatePose;
//   //     }
//   // }

//   /**
//    * Helper method for comparing vision pose against odometry pose. Does not account for difference
//    * in rotation. Will return false vision if it sees no targets or if the vision estimated pose is
//    * too far from the odometry estimate
//    *
//    * @return whether or not pose should be added to estimate or not
//    */
//   public boolean isEstimateReady(Pose2d pose) {
//     /* Disregard Vision if there are no targets in view */
//     if (!Vision.getInstance().visionAccurate(pose)) { // visionAccurate method sees if Apriltags present in Vision.java
//       return false;
//     }

//     // Disregard measurements too far away from odometry
//     // this can be tuned to find a threshold that helps us remove jumping vision
//     // poses
//     return (Math.abs(pose.getX() - odometryPose.getX()) <= VisionConfig.DIFFERENCE_CUTOFF_THRESHOLD)
//         && (Math.abs(pose.getY() - odometryPose.getY()) <= VisionConfig.DIFFERENCE_CUTOFF_THRESHOLD);
//   }

//   /** Sets the Odometry Pose to the given pose */
//   public void setOdometryPose(Pose2d pose) {
//     odometryPose = pose;
//   }

//   /** Returns the Odometry Pose from drivetrain */
//   public Pose2d getOdometryPose() {
//     return odometryPose;
//   }

//   /** Sets the desired pose of the robot */
//   public void setDesiredPose(Pose2d pose) {
//     desiredPose = pose;
//   }

//   /** Sets the estimated pose to the given pose */
//   public void setEstimatedPose(Pose2d pose) {
//     estimatePose = pose;
//   }

//   /** Updates the field relative position of the robot. */
//   public void updateOdometryEstimate() {
//     poseEstimator.update(drivetrain.getYaw(), drivetrain.getModulePositions());
//   }

//   /**
//    * @see edu.wpi.first.math.estimator.PoseEstimator#addVisionMeasurement(Pose2d, double)
//    */
//   public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
//     poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
//   }

//   /**
//    * Reset the pose estimator location and Drivetrain odometry - NEEDS TO BE TESTED
//    *
//    * @param poseMeters
//    */
//   public void resetPoseEstimate(Pose2d poseMeters) {
//     poseEstimator.resetPosition(drivetrain.getYaw(), drivetrain.getModulePositions(), poseMeters);
//     drivetrain.resetOdometry(poseMeters);
//   }

//   public void resetHeading(Rotation2d angle) {
//     drivetrain.resetOdometry(new Pose2d(drivetrain.getPose().getTranslation(), angle));
//     resetPoseEstimate(new Pose2d(estimatePose.getTranslation(), angle));
//   }

//   public void resetLocationEstimate(Translation2d translation) {
//     resetPoseEstimate(new Pose2d(translation, new Rotation2d(0)));
//   }

//   /**
//    * Gets the pose of the robot at the current time as estimated by the poseEstimator. This includes
//    * vision and odometry combined together.
//    *
//    * @return The estimated robot pose in meters.
//    */
//   public Pose2d getPosition() {
//     return poseEstimator.getEstimatedPosition();
//   }

//   /**
//    * Get the heading of the robot estimated by the poseEstimator. Use this in most places we would
//    * use the gyro.
//    *
//    * @return
//    */
//   public Rotation2d getHeading() {
//     return estimatePose.getRotation();
//   }

//   public Translation2d getLocation() {
//     return estimatePose.getTranslation();
//   }

//   public Pose2d getEstimatedPose() {
//     return estimatePose;
//   }

//   /**
//    * Creates a vector of standard deviations for the states. Standard deviations of model states.
//    * Increase these numbers to trust your model's state estimates less.
//    *
//    * @param x in meters
//    * @param y in meters
//    * @param theta in degrees
//    * @return the Vector of standard deviations need for the poseEstimator
//    */
//   public Vector<N3> createStateStdDevs(double x, double y, double theta) {
//     return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
//   }

//   /**
//    * Creates a vector of standard deviations for the local measurements. Standard deviations of
//    * encoder and gyro rate measurements. Increase these numbers to trust sensor readings from
//    * encoders and gyros less.
//    *
//    * @param theta in degrees per second
//    * @param s std for all module positions in meters per sec
//    * @return the Vector of standard deviations need for the poseEstimator
//    */
//   public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
//     return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
//   }

//   /**
//    * Creates a vector of standard deviations for the vision measurements. Standard deviations of
//    * global measurements from vision. Increase these numbers to trust global measurements from
//    * vision less.
//    *
//    * @param x in meters
//    * @param y in meters
//    * @param theta in degrees
//    * @return the Vector of standard deviations need for the poseEstimator
//    */
//   public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
//     return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
//   }

//   /**
//    * Commnad to reset odometry of drivetrain and pose esimator to the one from vision
//    * @return a command to reset the Pose Estimator and Drivetrain to the vision pose
//    */
//   public Command resetOdometryVisionCommand(){
//     return new InstantCommand(() -> resetPoseEstimate(Vision.getInstance().visionBotPose()));
//   }

//   public Command tempResetOdometryCOmmand(){
//     return new InstantCommand(() -> resetPoseEstimate(new Pose2d(2, 5.52, new Rotation2d(0))));
//   }
  
// }