package frc.robot.targeting.mode.commands;

import frc.robot.targeting.mode.*;

public class ShootingModeLobPositionCommand extends ShootingModeCommandBase {

    public ShootingModeLobPositionCommand(
            ShootingMode shootingMode) {
        super(shootingMode, "Shoot Lob Position", ShootingModeEnum.LobPosition);
    }
}
