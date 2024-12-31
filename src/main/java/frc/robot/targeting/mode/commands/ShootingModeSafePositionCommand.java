package frc.robot.targeting.mode.commands;

import frc.robot.targeting.mode.*;

public class ShootingModeSafePositionCommand extends ShootingModeCommandBase {

    public ShootingModeSafePositionCommand(
            ShootingMode shootingMode) {
        super(shootingMode, "Shoot Safe Position", ShootingModeEnum.SafePosition);
    }
}
