package frc.robot.targeting.mode.commands;

import frc.robot.targeting.mode.*;

public class ShootingModeManualCommand extends ShootingModeCommandBase {

    public ShootingModeManualCommand(
            ShootingMode shootingMode) {
        super(shootingMode, "Shoot Manual", ShootingModeEnum.Manual);
    }
}
