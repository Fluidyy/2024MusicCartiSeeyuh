package frc.robot.subsystems.lookuptable;

import java.util.Collections;
import java.util.List;

public class lookuptableabs {
    ShooterConfig shooterConfig;
    private static lookuptableabs instance = new lookuptableabs();

    public static lookuptableabs getInstance() {
        return instance;
    }

    public lookuptableabs() {
        shooterConfig = new ShooterConfig();
        // Initialization of shooterConfig with ShoterPreset objects...
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-7.0021484375, 0.8, 0.8, 1.0601)); // Distance -> Bumper

        shooterConfig.getShooterConfigs().add(new ShoterPreset(-5.6853515625, 0.8, 0.8, 1.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-4.4330078125, .8, 0.8, 2.1)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.9232421875, 0.8, 0.8, 2.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.50625, 0.8, 0.8, 3)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShoterPreset(-3.427734375, 0.8,-0.8, 3.5)); // Distance -> Bumper

        shooterConfig.getShooterConfigs().add(new ShoterPreset(-2.08681640625, .8, .8, 4)); // Distance -> Bumper
        Collections.sort(shooterConfig.getShooterConfigs());
    }

    public ShoterPreset getShooterPreset(double DistanceFromTarget) {
        List<ShoterPreset> configs = shooterConfig.getShooterConfigs();
        if (configs.isEmpty()) {
            // Handle the case where there are no presets
            return null; // or a default ShoterPreset, depending on your requirements
        }

        ShoterPreset closestBelow = null; // Closest preset below the target distance
        ShoterPreset closestAbove = null; // Closest preset above the target distance

        // Iterate through the sorted list to find the presets closest to the target distance
        for (ShoterPreset preset : configs) {
            if (preset.getDistance() <= DistanceFromTarget) {
                closestBelow = preset; // The last preset below the target distance
            } else {
                closestAbove = preset; // The first preset above the target distance
                break; // No need to continue through the list
            }
        }

        // Decision logic to choose between the closest below or above preset
        if (closestBelow == null) {
            // If there's no preset below the target, choose the closest above
            return closestAbove;
        } else if (closestAbove == null) {
            // If there's no preset above the target, choose the closest below
            return closestBelow;
        } else {
            // Both presets found, choose the one that is closer to the target distance
            double diffBelow = DistanceFromTarget - closestBelow.getDistance();
            double diffAbove = closestAbove.getDistance() - DistanceFromTarget;
            return (diffBelow <= diffAbove) ? closestBelow : closestAbove;
        }
    }


}
