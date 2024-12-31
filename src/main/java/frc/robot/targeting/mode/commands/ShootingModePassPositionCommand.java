package frc.robot.targeting.mode.commands;

import frc.robot.targeting.mode.ShootingMode;
import frc.robot.targeting.mode.ShootingModeEnum;

public class ShootingModePassPositionCommand extends ShootingModeCommandBase {

    public ShootingModePassPositionCommand(
            ShootingMode shootingMode) {
        super(shootingMode, "Shoot Pass Position", ShootingModeEnum.PassPosition);
    }
}
