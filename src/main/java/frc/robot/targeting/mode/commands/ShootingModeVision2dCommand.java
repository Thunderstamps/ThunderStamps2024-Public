package frc.robot.targeting.mode.commands;

import frc.robot.targeting.mode.*;

public class ShootingModeVision2dCommand extends ShootingModeCommandBase {
    
    public ShootingModeVision2dCommand(
            ShootingMode shootingMode) {
        super(shootingMode, "Shoot Vision 2D", ShootingModeEnum.Vision2D);
    }
}
