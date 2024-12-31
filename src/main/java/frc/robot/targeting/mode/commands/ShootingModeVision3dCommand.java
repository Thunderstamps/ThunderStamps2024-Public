package frc.robot.targeting.mode.commands;

import frc.robot.targeting.mode.*;

public class ShootingModeVision3dCommand extends ShootingModeCommandBase {
    
    public ShootingModeVision3dCommand(
            ShootingMode shootingMode) {
        super(shootingMode, "Shoot Vision 3D", ShootingModeEnum.Vision3D);
    }
}
