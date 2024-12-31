package frc.robot.targeting.mode;

import frc.robot.communications.NetworkTableComms;

public class ShootingMode {
    
    private ShootingModeEnum shootingModeEnum;
    private NetworkTableComms nt;

    public ShootingMode(
            ShootingModeEnum defaultMode,
            NetworkTableComms nt) {
        this.nt = nt;
        this.SetMode(defaultMode);
    }

    public boolean IsManual() {
        return this.shootingModeEnum == ShootingModeEnum.Manual;
    }

    public boolean IsSafePosition() {
        return this.shootingModeEnum == ShootingModeEnum.SafePosition;
    }

    public boolean IsLobPosition() {
        return this.shootingModeEnum == ShootingModeEnum.LobPosition;
    }

    public boolean IsPassPosition() {
        return this.shootingModeEnum == ShootingModeEnum.PassPosition;
    }

    public ShootingModeEnum GetMode() {
        return this.shootingModeEnum;
    }

    public void SetMode(ShootingModeEnum newMode) {
        switch (newMode) {
            case Manual:
                this.nt.setShooterModeManual();
                break;
            case SafePosition:
                this.nt.setShooterModeSafePosition();
                break;
            case LobPosition:
                this.nt.setShooterModeLobPosition();
                break;
            case PassPosition:
                this.nt.setShooterModePassPosition();
                break;
            case Vision2D:
                this.nt.setShooterModeVision2D();
                break;
            case Vision3D:
                this.nt.setShooterModeVision3D();
                break;
        }
        this.shootingModeEnum = newMode;
    }
}
