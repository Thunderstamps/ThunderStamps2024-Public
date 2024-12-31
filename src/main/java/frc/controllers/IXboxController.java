package frc.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public interface IXboxController {

    PovService getPovService();

    boolean getXButton();

    boolean getBackButtonPressed();
    boolean getStartButtonPressed();
    boolean getAButtonPressed();
    boolean getBButtonPressed();
    boolean getXButtonPressed();
    boolean getYButtonPressed();

    boolean getLeftBumperPressed();
    boolean getRightBumperPressed();
    double getLeftTriggerAxis();
    double getRightTriggerAxis();
    double getLeftX();
    double getRightX();
    double getLeftY();
    double getRightY( );
    void rumble();

    void printRawAnalogs();

    CommandXboxController getCommandXboxController();
    XboxController getXboxController();
}