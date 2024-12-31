package frc.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class XboxControllerBase implements IXboxController {

    private final CommandXboxController commandXboxController;
    private final XboxController xboxController;
    private final PovService povService;

    private double leftStickOffsetX = 0.0;
    private double leftStickOffsetY = 0.0;
    private double rightStickOffsetX = 0.0;
    private double rightStickOffsetY = 0.0;

    private double rumbleStart_sec = Timer.getFPGATimestamp();
    private boolean rumbling = false;

    protected XboxControllerBase(int port) {
        this.commandXboxController = new CommandXboxController(port);
        this.xboxController = this.commandXboxController.getHID();
        this.povService = new PovService(xboxController);
    }

    public PovService getPovService() {
        return this.povService;
    }

    public boolean getXButton() {
        return this.xboxController.getXButton();
    }

    public boolean getBackButtonPressed() {
        return this.xboxController.getBackButtonPressed();
    }

    public boolean getStartButtonPressed() {
        return this.xboxController.getStartButtonPressed();
    }

    public boolean getAButtonPressed() {
        return this.xboxController.getAButtonPressed();
    }

    public boolean getBButtonPressed() {
        return this.xboxController.getBButtonPressed();
    }

    public boolean getXButtonPressed() {
        return this.xboxController.getXButtonPressed();
    }

    public boolean getYButtonPressed() {
        return this.xboxController.getYButtonPressed();
    }

    public boolean getLeftBumperPressed() {
        return this.xboxController.getLeftBumperPressed();
    }

    public boolean getRightBumperPressed() {
        return this.xboxController.getRightBumperPressed();
    }

    public double getLeftTriggerAxis() {
        return this.xboxController.getLeftTriggerAxis();
    }

    public double getRightTriggerAxis() {
        return this.xboxController.getRightTriggerAxis();
    }

    protected void setLeftStickOffsetX(double offset) {
        this.leftStickOffsetX = offset;
    }

    protected void setLeftStickOffsetY(double offset) {
        this.leftStickOffsetY = offset;
    }

    protected void setRightStickOffsetX(double offset) {
        this.rightStickOffsetX = offset;
    }

    protected void setRightStickOffsetY(double offset) {
        this.rightStickOffsetY = offset;
    }

    public double getLeftX() {
        stopRumbling();
        var offset = 0.0;
        offset = this.leftStickOffsetX;
        var result = this.commandXboxController.getLeftX() + offset;
        return limitRange(result);
    }
    public double getRightX() {
        stopRumbling();
        var offset = 0.0;
        offset = this.rightStickOffsetX;
        var result = this.commandXboxController.getRightX() + offset;
        return limitRange(result);
    }


    public double getLeftY() {
        stopRumbling();
        var offset = 0.0;
        offset = this.leftStickOffsetY;
        var result = this.commandXboxController.getLeftY() + offset;
        return limitRange(result);
    }

    public double getRightY() {
        stopRumbling();
        var offset = 0.0;
        offset = this.rightStickOffsetY;
        var result = this.commandXboxController.getRightY() + offset;
        return limitRange(result);
    }

    private double limitRange(double input) {
        var result = input;
        if(result > 1.0) {
            result = 1.0;
        }
        if(result < -1.0) {
            result = -1.0;
        }
        return result;
    }

    public void printRawAnalogs() {
        printAnalog("LX", this.commandXboxController.getLeftX());
        printAnalog("LY", this.commandXboxController.getLeftY());
        printAnalog("RX", this.commandXboxController.getRightX());
        printAnalog("RY", this.commandXboxController.getRightY());
        System.out.println();
    }

    private void printAnalog(String name, double value) {
        System.out.print(name);
        System.out.print(":");
        System.out.printf("%.2f", value);
        System.out.print(", ");
    }

    public void rumble() {
        this.rumbleStart_sec = Timer.getFPGATimestamp();
        this.rumbling = true;
        this.xboxController.setRumble(RumbleType.kLeftRumble, 1.0);
    }

    private void stopRumbling() {
        if(this.rumbling) {
            var duration_sec = Timer.getFPGATimestamp() - this.rumbleStart_sec;
            if(duration_sec >= 0.5) {
                this.rumbling = false;
                this.xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
                this.xboxController.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }

    }

    @Override
    public CommandXboxController getCommandXboxController() {
        return this.commandXboxController;
    }

    @Override
    public XboxController getXboxController() {
        return this.xboxController;
    }
}