package frc.robot.subsystems.lookuptable;

/* Shooter preset which is used in the vision lookup table */

public class ShoterPreset implements Comparable<ShoterPreset> {
    private double m_armAngle;
    private double m_leftShooter;
    private double m_rightShooter;
    private double m_distance;

    public ShoterPreset(double armAngle, double leftShooter, double rightShooter, double distance) {
        this.m_armAngle = armAngle;
        this.m_leftShooter = leftShooter;
        this.m_rightShooter = rightShooter;
        this.m_distance = distance;
    }

    public double getArmAngle() {
        return m_armAngle;
    }

    public double getLeftShooter() {
        return m_leftShooter;
    }

    public double getRightShooter() {
        return m_rightShooter;
    }

    public double getDistance() {
        return m_distance; 
    }

    public void setArmAngle(double armAngle) {
        this.m_armAngle = armAngle;
    }

    public void setLeftShooter(double leftShooter) {
        this.m_leftShooter = leftShooter;
    }

    public void setRightShooter(double rightShooter) {
        this.m_rightShooter = rightShooter;
    }

    public void setDistance(double distance) {
        this.m_distance = distance;
    }

    @Override
    public int compareTo(ShoterPreset pPreset) {
        return Double.compare(this.getDistance(), pPreset.getDistance());
    }
}