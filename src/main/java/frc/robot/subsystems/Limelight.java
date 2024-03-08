package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Limelight extends SubsystemBase {
    private final DoubleArraySubscriber tagPoseTopic;
    private NetworkTable table;
    private double[] tagPose;
    private int updates;
    private static final double LIMELIGHT_HEIGHT = 10;
    private static final double LIMELIGHT_MOUNT_ANGLE = 0; // degrees, to be measured
    private static final double TARGET_HEIGHT = 30; // inches, the height of the target
    private static final double ARM_LENGTH = 21; // inches, length from pivot to arm end

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
        tagPose = new double[6];

    
    }

    @Override
    public void periodic() {
        refreshValues();
    }

    public int getUpdates() {
        return updates;
    }

    // X+ is to the right if you are looking at the tag
    public double getRX() {
        refreshValues();
        return tagPose[0];
    }

    // Y+ is upwards
    public double getRY() {
        refreshValues();
        return tagPose[1];
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public double getRZ() {
        refreshValues();
        return tagPose[2];
    }

    public double getPitch() {
        refreshValues();
        return tagPose[3];
    }

    public double getYaw() {
        refreshValues();
        return tagPose[4];
    }

    public double getRoll() {
        refreshValues();
        return tagPose[5];
    }
    public double calculateDistance(double ty, double limelightHeight, double mountingAngle, double targetHeight) {
        double angleToTarget = Math.toRadians(mountingAngle + ty);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }

    public void refreshValues() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPose = tagPoseTopic.get(new double[6]);
        updates++;

    
    }

    
    
}
   