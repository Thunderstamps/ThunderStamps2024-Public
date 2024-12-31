// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.unmanaged.Unmanaged;

public class Robot extends LoggedRobot {
  
  private RobotContainer robotContainer; 
  private Command autonomousCommand;
    
  @Override
  public void robotInit() {
    
    // debugging requested by CTRE
    // should be 0x180300 or 1573632 for Phoenix 24.3.0
    var phoenixVersion = Unmanaged.getPhoenixVersion();
    System.out.printf("Phoenix Version: %s should be 1573632%n", phoenixVersion);

    AdvantageKit.initializeLogging(this);
    this.robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    this.robotContainer.RunRobotPeriodicBeforeCommandScheduler();
    CommandScheduler.getInstance().run();
    this.robotContainer.RunRobotPeriodicAfterCommandScheduler();
  }

  @Override
  public void autonomousInit() {
    this.robotContainer.AutonomousInit();
    this.autonomousCommand = this.robotContainer.getAutonomousCommand();
    this.autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    this.robotContainer.RunAutonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    if(this.autonomousCommand != null) {
      this.autonomousCommand.cancel();
      this.autonomousCommand = null;
    }
    this.robotContainer.TeleopInit();
    
  }

  @Override
  public void teleopPeriodic() {
    this.robotContainer.RunTeleopPeriodic();
  }

  @Override
  public void disabledInit() {
    this.robotContainer.DisabledInit();
  }

  @Override
  public void disabledPeriodic() {
    this.robotContainer.RunDisabledPeriodic();
  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  private final LinearFilter voltageFilter = LinearFilter.singlePoleIIR(0.03, Constants.SCAN_TIME_S);

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    switch (AdvantageKit.getCurrentMode()) {
      case SIM:
      case SIM_LOGGED:
        var totalCurrent_A = robotContainer.getTotalCurrent_A();
        var voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent_A);
        var smoothedVoltage = voltageFilter.calculate(voltage);
        RoboRioSim.setVInVoltage(smoothedVoltage);
        break;

      default:
        break;
    }
  }
}
