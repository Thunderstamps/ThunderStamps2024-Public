package frc.robot.targeting.mode.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.targeting.mode.*;

public abstract class ShootingModeCommandBase extends Command {
    
    private ShootingMode shootingMode;
    private ShootingModeEnum shootingModeEnum;

    protected ShootingModeCommandBase(
            ShootingMode shootingMode, 
            String commandName, 
            ShootingModeEnum shootingModeEnum) {
        this.shootingMode = shootingMode;
        this.shootingModeEnum = shootingModeEnum;
        this.setName(commandName);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.shootingMode.SetMode(this.shootingModeEnum);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
