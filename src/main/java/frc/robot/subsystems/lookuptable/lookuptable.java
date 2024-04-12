package frc.robot.subsystems.lookuptable;

import java.util.Collections;
import java.util.List;

public class lookuptable {
    ShooterConfig shooterConfig;

    private static lookuptable instance = new lookuptable();

    public static lookuptable getInstance() {
        return instance;
    }
    public lookuptable() {
        shooterConfig = new ShooterConfig();
                shooterConfig.getShooterConfigs().add(new ShoterPreset(-7.0021484375, 0.8, 0.8, 1.0601)); // Distance -> Bumper
                shooterConfig.getShooterConfigs().add(new ShoterPreset(-5.49853515625, 0.8, 0.8, 1.25)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-4.79638671875, 0.8, 0.8, 1.5)); // Distance -> Bumper
                shooterConfig.getShooterConfigs().add(new ShoterPreset(-4.27197265625, 0.8, 0.8, 1.75)); // Distance -> Bumper
                          shooterConfig.getShooterConfigs().add(new ShoterPreset(-4.21484375, 0.8, 0.8, 2)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.7724609375, .8, 0.8, 2.25)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.9232421875, 0.8, 0.8, 2.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.50625, 0.8, 0.8, 3)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.427734375, 0.8,-0.8, 3.5)); // Distance -> Bumper

        shooterConfig.getShooterConfigs().add(new ShoterPreset(-2.08681640625, .8, .8, 4)); // Distance -> Bumper


        Collections.sort(shooterConfig.getShooterConfigs());
    }

    /*
     * Obtains a shooter preset from a given target distance
     * @param DistanceFromTarget measured distance to the shooting target
     * @return new shooter preset for given distance
     */
    public ShoterPreset getShooterPreset(double DistanceFromTarget) {
        int endIndex = shooterConfig.getShooterConfigs().size()-1;

        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured distance is shorter
         * select the lookup table entry with the shortest distance
         */
        if(DistanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()){
            return shooterConfig.getShooterConfigs().get(0);
        }

        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured distance is larger
         * select the lookup table entry with the largest distance
         */
        if(DistanceFromTarget >= shooterConfig.getShooterConfigs().get(endIndex).getDistance()){
            return shooterConfig.getShooterConfigs().get(endIndex);
        }
        /*
         * If the measured distance falls somewhere within the lookup table perform a binary seqarch within the lookup
         * table
         */
        return binarySearchDistance(shooterConfig.getShooterConfigs(),0, endIndex, DistanceFromTarget);
    }

    /*
     * Perform fast binary search to find a matching shooter preset. if no matching preset is found it interpolates a
     * new shooter preset based on the two surrounding table entries.
     * 
     * @param ShooterConfigs: the table containing the shooter presets
     * @param StartIndex: Starting point to search
     * @param EndIndex: Ending point to search
     * @param Distance: Distance for which we need to find a preset
     * 
     * @return (Interpolated) shooting preset
     */
    private ShoterPreset binarySearchDistance(List<ShoterPreset> ShooterConfigs, int StartIndex, int EndIndex, double Distance) {
        int mid = StartIndex + (EndIndex - StartIndex) / 2;
        double midIndexDistance = ShooterConfigs.get(mid).getDistance();

        // If the element is present at the middle
        // return itself
        if (Distance == midIndexDistance) {
            return ShooterConfigs.get(mid);
        }
        // If only two elements are left
        // return the interpolated config
        if (EndIndex - StartIndex == 1) {
            double percentIn = (Distance - shooterConfig.getShooterConfigs().get(StartIndex).getDistance()) / 
                (
                    shooterConfig.getShooterConfigs().get(EndIndex).getDistance() - 
                        shooterConfig.getShooterConfigs().get(StartIndex).getDistance()
                );
            return interpolateShooterPreset(shooterConfig.getShooterConfigs().get(StartIndex), shooterConfig.getShooterConfigs().get(EndIndex), percentIn);
        }
        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (Distance < midIndexDistance) {
            return binarySearchDistance(ShooterConfigs, StartIndex, mid, Distance);
        }
        // Else the element can only be present in right subarray
        return binarySearchDistance(ShooterConfigs, mid, EndIndex, Distance);
    }

    /*
     * Obtain a new shooter preset by interpolating between two existing shooter presets
     * 
     * @param StartPreset: Starting preset for interpolation
     * @param EndPreset: Ending preset for interpolation
     * @param PercentIn: Amount of percentage between the two values the new preset needs to be
     * 
     * @return new interpolated shooter preset
     */ 
    private ShoterPreset interpolateShooterPreset(ShoterPreset StartPreset, ShoterPreset EndPreset, double PercentIn) {
        // Apply an exponential effect by squaring the PercentIn for acceleration
        
        double exponentialIn = Math.pow(PercentIn, 2);
    
        double armAngle = StartPreset.getArmAngle() + (EndPreset.getArmAngle() - StartPreset.getArmAngle()) * exponentialIn;
        double leftShooter = StartPreset.getLeftShooter() + (EndPreset.getLeftShooter() - StartPreset.getLeftShooter()) * exponentialIn;
        double rightShooter = StartPreset.getRightShooter() + (EndPreset.getRightShooter() - StartPreset.getRightShooter()) * exponentialIn;
        double distance = StartPreset.getDistance() + (EndPreset.getDistance() - StartPreset.getDistance()) * exponentialIn;
    
        return new ShoterPreset(armAngle, leftShooter, rightShooter, distance);
    }

    /*
     * MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION
     * @param pShooterConfig a sorted shooter config
     */
    public void setShooterConfig(ShooterConfig pShooterConfig) {
        this.shooterConfig = pShooterConfig;
    }
}