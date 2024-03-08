// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// // import frc.robot.Robot;

// public class PoseTelemetry {
//   PoseEstimator pose;
//   Field2d field = new Field2d();

//   public PoseTelemetry(PoseEstimator pose) {
//     this.pose = pose;

//     createField();
//   }

//   private void createField() {
//     SmartDashboard.putData("Field", field);
//   }

//   public void updatePoseOnField(String name, Pose2d pose) {
//     field.getObject(name).setPose(pose);
//     // Robot.log.logger.recordOutput(name, pose);
//   }

//   /**
//    * get the current field2d object
//    *
//    * @return field2d object
//    */
//   public Field2d getField() {
//     return field;
//   }

//   public void testMode() {}
// }