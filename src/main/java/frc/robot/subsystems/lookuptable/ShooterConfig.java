package frc.robot.subsystems.lookuptable;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.lookuptable.ShoterPreset;

public class ShooterConfig {
    private List<ShoterPreset> shooterConfigs;

    public ShooterConfig() {
        shooterConfigs = new ArrayList<ShoterPreset>();
    }

    public ShooterConfig(ArrayList<ShoterPreset> pShooterConfigs) {
        this.shooterConfigs = pShooterConfigs;
    }

    public List<ShoterPreset> getShooterConfigs() {
        return shooterConfigs;
    }
}