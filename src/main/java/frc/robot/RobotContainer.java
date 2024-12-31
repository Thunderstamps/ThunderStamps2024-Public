package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.controllers.*;
import frc.robot.communications.*;
import frc.robot.note.NoteDetector;
import frc.robot.note.NoteDetectorIO;
import frc.robot.note.NoteDetectorIOReal;
import frc.robot.note.commands.*;
import frc.robot.subsystems.angle.*;
import frc.robot.subsystems.angle.commands.*;
import frc.robot.subsystems.commands.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.indexer.commands.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.commands.*;
import frc.robot.subsystems.intake.commands.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;
import frc.robot.subsystems.swerve.drive.DriveControllerIO;
import frc.robot.subsystems.swerve.drive.DriveControllerIOReal;
import frc.robot.subsystems.swerve.gyros.*;
import frc.robot.subsystems.swerve.steer.SteeringControllerIO;
import frc.robot.subsystems.swerve.steer.SteeringControllerIOReal;
import frc.robot.subsystems.swerve.util.SwerveUtil;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.commands.*;
import frc.robot.subsystems.tramp.*;
import frc.robot.subsystems.tramp.commands.*;
import frc.robot.subsystems.wallclimb.*;
import frc.robot.subsystems.wallclimb.commands.*;
import frc.robot.targeting.*;
import frc.robot.targeting.mode.*;
import frc.robot.targeting.mode.commands.*;
import frc.robot.targeting.providers.limelight.Vision2DIO;
import frc.robot.targeting.providers.limelight.Vision2DIOReal;
import frc.robot.utilities.canivore.CanivoreIO;
import frc.robot.utilities.canivore.CanivoreIOReal;
import frc.robot.utilities.canivore.CanivoreSubsystem;

public class RobotContainer {
    private final NetworkTableComms nt = new NetworkTableComms();
    private final IXboxController xboxController = new XboxControllerSCUF(0);
    private final CommandXboxController commandXboxController;
    private final XboxController xbox;
    private final CommandJoystick joystick = new CommandJoystick(1);

    private final Trigger resetGyroButton;
    private final Trigger enableGyroButton;
    private final Trigger disableGyroButton;
    private final Trigger shootButton;
    private final Trigger runIntakeButton;
    private final Trigger stopIntakeButton;
    private final Trigger unjamIntakeButton;
    private final Trigger transferButton;
    private final Trigger prepareToClimbButton;
    private final Trigger cancelClimbButton;
    private final Trigger enableClimbButton;
    private final Trigger climbButton;
    private final Trigger modeVision3dButton;
    private final Trigger modeVision2dButton;
    private final Trigger modeLobPositionButton;
    private final Trigger modeSafePositionButton;
    private final Trigger modePassPositionButton;
    private final Trigger enableAssistModeButton;
    private final Trigger disableAssistModeButton;
    private final Trigger reverseTrampButton;
    private final Trigger forwardTrampButton;
    
    private final ShootingMode shootingMode = new ShootingMode(ShootingModeEnum.Vision2D, nt);
    private final TargetingController targetingController;
    private final NoteDetector noteDetector;
    //private final Vision3D vision3D = new Vision3D(nt);
    private final IGyro gyro;
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final AngleSubsystem angleSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final WallClimbSubsystem wallClimbSubsystem;
    private final TrampSubsystem trampSubsystem;
    private final CanivoreSubsystem canivoreSubsystem;
    
    private final LoggedDashboardChooser<Command> autoChooser;
    //private final Command autoCommand;
    
    public RobotContainer() {
        this.commandXboxController = this.xboxController.getCommandXboxController();
        this.xbox = this.xboxController.getXboxController();

        this.resetGyroButton = this.commandXboxController.back();
        this.enableGyroButton = this.joystick.button(12);
        this.disableGyroButton = this.joystick.button(11);
        this.shootButton = this.commandXboxController.a();
        this.runIntakeButton = this.commandXboxController.x();
        this.stopIntakeButton = this.commandXboxController.start();
        this.unjamIntakeButton = this.commandXboxController.y();
        this.transferButton = this.commandXboxController.b();
        this.prepareToClimbButton = this.commandXboxController.povUp();
        this.cancelClimbButton = this.commandXboxController.povLeft()
            .or(this.commandXboxController.povRight());
        this.enableClimbButton = this.joystick.trigger();
        this.climbButton = this.commandXboxController.povDown();
        this.modeVision3dButton = this.joystick.button(2); // thumb
        this.modeVision2dButton = this.joystick.button(3); // lower left
        this.modeLobPositionButton = this.joystick.button(4); // lower right
        this.modeSafePositionButton = this.joystick.button(5); // upper left
        this.modePassPositionButton = this.joystick.button(6); // upper right
        this.enableAssistModeButton = this.joystick.button(8);
        this.disableAssistModeButton = this.joystick.button(7);
        this.reverseTrampButton = this.joystick.button(9);
        this.forwardTrampButton = this.joystick.button(10);

        // Primary NavX2 has a ScaleFactor of 0.9125
        // Backup Navx2 has a ScaleFactor of ?
        // If installing the NavX classic board, use the NavXMxpGyro class instead of the NavX2MxpGyro class

        switch (AdvantageKit.getCurrentMode()) {
            case REAL:
                //this.gyro = new NavX2MxpGyro(0.9125, new NavX2MxpGyroIOReal());
                this.gyro = new Pigeon2Gyro(new Pigeon2GyroIOReal());
                this.targetingController = new TargetingController(shootingMode, nt, new Vision2DIOReal(nt)); 
                this.noteDetector = new NoteDetector(nt, new NoteDetectorIOReal());
                this.angleSubsystem = new AngleSubsystem(nt, gyro, new AngleIOReal(nt));
                this.canivoreSubsystem = new CanivoreSubsystem(new CanivoreIOReal());
                this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal(nt));
                this.indexerSubsystem = new IndexerSubsystem(nt, new IndexerIOReal(nt));
                this.intakeSubsystem = new IntakeSubsystem(nt, new IntakeIOReal(nt));
                this.shooterSubsystem = new ShooterSubsystem(
                    nt, 
                    targetingController,
                    new ShooterSideIOReal(Constants.ShooterLeftCanbus),
                    new ShooterSideIOReal(Constants.ShooterRightCanbus));
                this.swerveSubsystem = new SwerveSubsystem(xboxController, gyro, nt, targetingController, 
                    new DriveControllerIOReal(Constants.SwerveModule1), 
                    new DriveControllerIOReal(Constants.SwerveModule2), 
                    new DriveControllerIOReal(Constants.SwerveModule3), 
                    new DriveControllerIOReal(Constants.SwerveModule4),
                    new SteeringControllerIOReal(Constants.SwerveModule1),
                    new SteeringControllerIOReal(Constants.SwerveModule2),
                    new SteeringControllerIOReal(Constants.SwerveModule3),
                    new SteeringControllerIOReal(Constants.SwerveModule4));
                this.trampSubsystem = new TrampSubsystem(nt, new TrampIOReal(nt));
                this.wallClimbSubsystem = new WallClimbSubsystem(nt, new WallClimbIOReal(nt));
                break;
        
            default:
                //this.gyro = new NavX2MxpGyro(0.9125, new NavX2MxpGyroIO() {});
                this.gyro = new Pigeon2Gyro(new Pigeon2GyroIO() {});
                this.targetingController = new TargetingController(shootingMode, nt, new Vision2DIO() {}); 
                this.noteDetector = new NoteDetector(nt, new NoteDetectorIO() {});
                this.angleSubsystem = new AngleSubsystem(nt, gyro, new AngleIO() {});
                this.canivoreSubsystem = new CanivoreSubsystem(new CanivoreIO() {});
                this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
                this.indexerSubsystem = new IndexerSubsystem(nt, new IndexerIO() {});
                this.intakeSubsystem = new IntakeSubsystem(nt, new IntakeIO() {});
                this.shooterSubsystem = new ShooterSubsystem(
                    nt, 
                    targetingController,
                    new ShooterSideIO() {},
                    new ShooterSideIO() {});
                this.swerveSubsystem = new SwerveSubsystem(xboxController, gyro, nt, targetingController, 
                    new DriveControllerIO() {}, 
                    new DriveControllerIO() {}, 
                    new DriveControllerIO() {}, 
                    new DriveControllerIO() {},
                    new SteeringControllerIO() {},
                    new SteeringControllerIO() {},
                    new SteeringControllerIO() {},
                    new SteeringControllerIO() {});
                this.trampSubsystem = new TrampSubsystem(nt, new TrampIO() {});
                this.wallClimbSubsystem = new WallClimbSubsystem(nt, new WallClimbIO() {});
                break;
        }

        this.initAutoNamedCommands(); // commands used by FRC PathPlanner

        // this must run after swerveSubsystem is created
        this.autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser()); // Loads all FRC PathPlanner auto routines
        var wheelTest = new ParallelCommandGroup(
            new MoveXDirectionAtSpeedCommand (swerveSubsystem, 10.0),
            new ShooterOffCommand(shooterSubsystem)
        );
        this.autoChooser.addOption("Wheel Test", wheelTest);

        SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

        //this.autoCommand = AutoBuilder.buildAuto("Red Source A");

        this.initShooterMode(); // creates buttons on the dashboard to select shooter mode

        // Comment any of these subsystems out to disable it from working
        this.initSwerve();
        this.initIntake();
        this.initIndexer();
        this.initAngle();
        this.initShooter();
        this.initElevator();
        this.initWallClimb();
        this.initTramp();

        var canTransfer = 
            this.indexerSubsystem.IsInitialized()
            && this.angleSubsystem.IsInitialized()
            && this.wallClimbSubsystem.IsInitialized()
            && this.intakeSubsystem.IsInitialized()
            && this.trampSubsystem.IsInitialized();
        this.transferButton
            .and(() -> canTransfer)
            .and(shooterSubsystem::IsNotClimbing)
            .onTrue(new TransferCommand(indexerSubsystem, angleSubsystem, wallClimbSubsystem, intakeSubsystem, trampSubsystem));
        
        var prepareToClimbCommand = new PrepareToClimbCommand(angleSubsystem, indexerSubsystem, shooterSubsystem);

        var canScoreTramp = 
            this.elevatorSubsystem.IsInitialized()
            && this.trampSubsystem.IsInitialized(); // don't really need wall climb, it just runs it to help the note go in the amp
        this.shootButton 
            .and(this.trampSubsystem::GetNotePresent)
            .and(() -> canScoreTramp)
            .and(() -> !prepareToClimbCommand.IsReadyToClimb())
            .and(shooterSubsystem::IsNotClimbing)
            .onTrue(new AmpScoreCommand(elevatorSubsystem, trampSubsystem, wallClimbSubsystem));

        var canPrepareToClimb =
            this.angleSubsystem.IsInitialized();
        this.prepareToClimbButton
            .and(() -> canPrepareToClimb)
            .onTrue(prepareToClimbCommand
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .until(this.cancelClimbButton));

        
        var canClimb =
            this.elevatorSubsystem.IsInitialized()
            && this.wallClimbSubsystem.IsInitialized() // indexer and shooter aren't needed, they just get stopped during climb, 
            && this.angleSubsystem.IsInitialized(); // ... and can climb without the tramp scorer
        var climbCommand = new ClimbCommand(
                elevatorSubsystem, 
                wallClimbSubsystem, 
                angleSubsystem, 
                indexerSubsystem, 
                shooterSubsystem, 
                trampSubsystem,
                swerveSubsystem)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        this.climbButton
            .and(this.enableClimbButton) // have to hold trigger on joystick to enable driver climb button
            .and(() -> canClimb)
            .and(prepareToClimbCommand::IsReadyToClimb)
            .onTrue(climbCommand);

    }

    void initAutoNamedCommands() {
        
        NamedCommands.registerCommand("stop", new StopCommand(swerveSubsystem));

        NamedCommands.registerCommand("runIntake", new RunIntakeUntilNote(intakeSubsystem, indexerSubsystem));
        NamedCommands.registerCommand("stopIntake", new StopIntakeCommand(intakeSubsystem));
        
        NamedCommands.registerCommand("trackNoteN4", new TrackNoteInAutoCommand(swerveSubsystem, noteDetector, NoteNumber.N4));
        NamedCommands.registerCommand("trackNoteN5", new TrackNoteInAutoCommand(swerveSubsystem, noteDetector, NoteNumber.N5));
        NamedCommands.registerCommand("trackNoteN6", new TrackNoteInAutoCommand(swerveSubsystem, noteDetector, NoteNumber.N6));
        NamedCommands.registerCommand("trackNoteN7", new TrackNoteInAutoCommand(swerveSubsystem, noteDetector, NoteNumber.N7));
        NamedCommands.registerCommand("trackNoteN8", new TrackNoteInAutoCommand(swerveSubsystem, noteDetector, NoteNumber.N8));

        NamedCommands.registerCommand("angleDefault", new AngleDefaultCommand(angleSubsystem));
        NamedCommands.registerCommand("waitForNote", new IndexerWaitForNoteCommand(indexerSubsystem));
        NamedCommands.registerCommand("waitForEmpty", new IndexerWaitForEmptyCommand(indexerSubsystem));
        NamedCommands.registerCommand("shoot", new IndexerShootCommand(indexerSubsystem));
        NamedCommands.registerCommand("shootStationary", new ShootStationaryCommand(swerveSubsystem, indexerSubsystem, intakeSubsystem, targetingController));

        NamedCommands.registerCommand("stopAndWaitForNote", new ParallelDeadlineGroup(
            new IndexerWaitForNoteCommand(indexerSubsystem), 
            new RunIntakeUntilNote(intakeSubsystem, indexerSubsystem),
            new StopCommand(swerveSubsystem)));

        NamedCommands.registerCommand("visionTarget", new ParallelCommandGroup(
            new AngleTargetCommand(angleSubsystem, targetingController),
            new ShooterRunToTargetCommand(shooterSubsystem),
            new StartEndCommand(
                () -> this.swerveSubsystem.SetAutoAim(true), 
                () -> this.swerveSubsystem.SetAutoAim(false))
        ));

        // Amp A *******************************************************************
        addTarget("targetRedAmpA1", 18.0, 155.0, 3200); // note we're holding
        addTarget("targetRedAmpA2", 17.7, 165.0); 
        addTarget("targetRedAmpA3", 17.7, 165.0); 
        addTarget("targetRedAmpA4", 15.0, -170.0); // last note from under the stage

        addTarget("targetBlueAmpA1", 18.0, 25.0, 3200);
        addTarget("targetBlueAmpA2", 17.7, 15.0);
        addTarget("targetBlueAmpA3", 17.7, 15.0); 
        addTarget("targetBlueAmpA4", 15.0, -10.0); 
        
        // Amp B *******************************************************************
        addTarget("targetRedAmpB1", 17.5, 170.0, 3200); // note we're holding
        
        addTarget("targetBlueAmpB1", 17.5, 10.0, 3200);

        // Centre A *******************************************************************
        addTarget("targetRedCentreA1", 5.8, 178.0);
        addTarget("targetRedCentreA2", 7.8, 178.0); // middle note
        addTarget("targetRedCentreA3", 7.7, -159.0); // note by protected/safe zone
        addTarget("targetRedCentreA4", 9.2, 150.0); // note by amp
        addTarget("targetRedCentreA5", 19.0, 160.0); // centre note
        
        addTarget("targetBlueCentreA1", 5.8, 2.0);
        addTarget("targetBlueCentreA2", 7.8, 2.0); // middle note
        addTarget("targetBlueCentreA3", 7.7, -21.0); // note by protected/safe zone
        addTarget("targetBlueCentreA4", 9.2, 30.0); // note by amp
        addTarget("targetBlueCentreA5", 19.0, 20.0); // centre note

        // Centre B *******************************************************************
        addTarget("targetRedCentreB5", 15.0, -170.0);// last note from under the stage
        
        addTarget("targetBlueCentreB5", 15.0, -10.0); 
        
        // Source A *******************************************************************
        addTarget("targetRedSourceA1", 15.7, -142.0, 3000.0);
        addTarget("targetRedSourceA2", 19.0, -141.0); 
        addTarget("targetRedSourceA3", 19.0, -143.0); 

        addTarget("targetBlueSourceA1", 15.7, -38.0, 3000.0);
        addTarget("targetBlueSourceA2", 19.0, -39.0); 
        addTarget("targetBlueSourceA3", 19.0, -37.0); 
        
        // Source B *******************************************************************
        addTarget("targetRedSourceB1", 16.15, -144.0);

        addTarget("targetBlueSourceB1", 16.15, -36.0);
    }

    void addTarget(String targetName, double distance_ft, double heading_deg, double speed_rpm) {
        var heading_rad = Math.toRadians(heading_deg);
        if(targetName.contains("Red")) {
            heading_rad -= Math.PI;
        }
        heading_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(heading_rad);
        NamedCommands.registerCommand(targetName, new SequentialCommandGroup(
            new SetTargetFieldOrientationCommand(swerveSubsystem, heading_rad),
            new DistanceTargetCommand(angleSubsystem, shooterSubsystem, indexerSubsystem, targetingController, distance_ft, speed_rpm)
        ));
    }

    void addTarget(String targetName, double distance_ft, double heading_deg) {
        var heading_rad = Math.toRadians(heading_deg);
        if(targetName.contains("Red")) {
            heading_rad -= Math.PI;
        }
        heading_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(heading_rad);
        NamedCommands.registerCommand(targetName, new SequentialCommandGroup(
            new SetTargetFieldOrientationCommand(swerveSubsystem, heading_rad),
            new DistanceTargetCommand(angleSubsystem, shooterSubsystem, indexerSubsystem, targetingController, distance_ft)
        ));
    }

    void initShooterMode() {
        var manualCommand = new ShootingModeManualCommand(this.shootingMode);
        var safePositionCommand = new ShootingModeSafePositionCommand(this.shootingMode);
        var lobPositionCommand = new ShootingModeLobPositionCommand(this.shootingMode);
        var passPositionCommand = new ShootingModePassPositionCommand(this.shootingMode);
        var vision2dCommand = new ShootingModeVision2dCommand(this.shootingMode);
        var vision3dCommand = new ShootingModeVision3dCommand(this.shootingMode);

        this.modeVision3dButton.onTrue(vision3dCommand);
        this.modeVision2dButton.onTrue(vision2dCommand);
        this.modeLobPositionButton.onTrue(lobPositionCommand);
        this.modeSafePositionButton.onTrue(safePositionCommand);
        this.modePassPositionButton.onTrue(passPositionCommand);

        this.nt.initializeShootingModeCommands(
            manualCommand, 
            safePositionCommand,
            lobPositionCommand, 
            passPositionCommand,
            vision2dCommand, 
            vision3dCommand);
    }

    void initSwerve(){

        var gyroEnableCommand = new GyroEnableCommand(this.swerveSubsystem);
        var gyroDisableCommand = new GyroDisableCommand(this.swerveSubsystem);

        this.enableGyroButton.onTrue(gyroEnableCommand);
        this.disableGyroButton.onTrue(gyroDisableCommand);

        var enableAssistModeCommand = new EnableAssistModeCommand(noteDetector);
        var disableAssistModeCommand = new DisableAssistModeCommand(noteDetector);

        this.enableAssistModeButton.onTrue(enableAssistModeCommand);
        this.disableAssistModeButton.onTrue(disableAssistModeCommand);

        this.nt.initializeSwerveCommands(
          gyroEnableCommand, 
          gyroDisableCommand,
          enableAssistModeCommand,
          disableAssistModeCommand);

        var zeroCommand = new GyroZeroCommand(this.swerveSubsystem);
        this.resetGyroButton.onTrue(zeroCommand);
        this.swerveSubsystem.setSubsystemInitialized();

        var trackNoteInTeleopCommand = new TrackNoteInTeleopCommand(swerveSubsystem, indexerSubsystem, trampSubsystem, intakeSubsystem, noteDetector);
        var trackNoteTrigger = new Trigger(intakeSubsystem::IsIntaking)
            .and(() -> DriverStation.isTeleopEnabled())
            .and(shooterSubsystem::IsNotClimbing);
        trackNoteTrigger.whileTrue(trackNoteInTeleopCommand);
    }

    void initIntake() {

        this.runIntakeButton
            .onTrue(new RunIntakeCommand(intakeSubsystem));

        var unjamIntakeCommand = new UnjamIntakeCommand(intakeSubsystem);
        this.unjamIntakeButton
            .onTrue(unjamIntakeCommand);

        this.stopIntakeButton
            .onTrue(new StopIntakeCommand(intakeSubsystem));

        var gotNoteTrigger = new Trigger(this.indexerSubsystem::GetNotePresent)
            .and(DriverStation::isTeleopEnabled);
        gotNoteTrigger
            .onTrue(new StopIntakeCommand(intakeSubsystem));

        var shootingTrigger = new Trigger(() -> this.indexerSubsystem.GetMode() == IndexerMode.Shoot);
        var reverseIntakeWhileShootingCommand = new StartEndCommand(
            () -> this.intakeSubsystem.RunReverse(), 
            () -> this.intakeSubsystem.Stop(), 
            this.intakeSubsystem);
        shootingTrigger
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(reverseIntakeWhileShootingCommand);

        this.intakeSubsystem.setSubsystemInitialized();
    }

    private void initIndexer() {
        // no default command, it's handled by a mode system inside the subsystem itself

        var indexerFireCommand = new IndexerShootCommand(indexerSubsystem);
        this.shootButton
            .and(shooterSubsystem::IsNotClimbing)
            .onTrue(indexerFireCommand);

        var gotNoteTrigger = new Trigger(this.indexerSubsystem::GetNotePresent)
            .and(DriverStation::isTeleopEnabled);
        var rumbleCommand = new StartEndCommand(
            () -> this.xbox.setRumble(RumbleType.kLeftRumble, 1.0),
            () -> this.xbox.setRumble(RumbleType.kLeftRumble, 0.0))
            .withTimeout(0.7);
        gotNoteTrigger.onTrue(rumbleCommand);

        this.indexerSubsystem.setSubsystemInitialized();
    }

    private void initAngle() {
        var defaultAngleCommand = new AngleDefaultCommand(angleSubsystem);
        this.angleSubsystem.setDefaultCommand(defaultAngleCommand);
        
        var gotNoteTrigger = new Trigger(this.indexerSubsystem::GetNotePresent);
        var angleDistanceCommand = new AngleTargetCommand(angleSubsystem, targetingController);
        gotNoteTrigger
            .and(DriverStation::isTeleopEnabled)
            .and(shooterSubsystem::IsNotClimbing)
            .whileTrue(angleDistanceCommand);

        this.angleSubsystem.setSubsystemInitialized();
    }

    private void initShooter() {
        var runShooterCommand = new ShooterRunToTargetCommand(shooterSubsystem);
        this.shooterSubsystem.setDefaultCommand(runShooterCommand);

        var idleTrigger = new Trigger(() -> !indexerSubsystem.GetNotePresent());
        idleTrigger
            .and(() -> DriverStation.isTeleopEnabled())
            .and(shooterSubsystem::IsNotClimbing)
            .whileTrue(new ShooterIdleCommand(shooterSubsystem));

        //if voltage goes too low (during acceleration) then idle it until it rises again
        var lowVoltageTrigger = new Trigger(() -> RobotController.getBatteryVoltage() < 8.5);
        lowVoltageTrigger
            .and(() -> DriverStation.isTeleopEnabled())
            .and(indexerSubsystem::GetNotePresent)
            .and(shooterSubsystem::IsNotClimbing)
            .onTrue(new ShooterCoastCommand(shooterSubsystem, 9.5));

        this.shooterSubsystem.setSubsystemInitialized();
    }

    public void initElevator() {
        this.elevatorSubsystem.setDefaultCommand(
            new ElevatorManualCommand(elevatorSubsystem, joystick));

        this.elevatorSubsystem.setSubsystemInitialized();
    }
    public void initWallClimb(){
        this.wallClimbSubsystem.setDefaultCommand(
            new WallClimbDefaultCommand (this.wallClimbSubsystem));

        this.wallClimbSubsystem.setSubsystemInitialized();
    }
    private void initTramp() {
        var defaultTrampCommand = new TrampDefaultCommand(trampSubsystem);
        trampSubsystem.setDefaultCommand(defaultTrampCommand);

        this.reverseTrampButton
            .and(shooterSubsystem::IsNotClimbing)
            .whileTrue(new TrampBackCommand(trampSubsystem));
        this.forwardTrampButton
            .and(shooterSubsystem::IsNotClimbing)
            .whileTrue(new TrampForwardCommand(trampSubsystem));

        this.trampSubsystem.setSubsystemInitialized();
    }

    public void RunRobotPeriodicBeforeCommandScheduler() {
        //this.vision3D.Periodic();

        // This reads the inputs from the hardware
        this.noteDetector.readInputs();
        this.canivoreSubsystem.readInputs();
        this.angleSubsystem.readInputs();
        this.elevatorSubsystem.readInputs();
        this.indexerSubsystem.readInputs();
        this.intakeSubsystem.readInputs();
        this.shooterSubsystem.readInputs();
        this.swerveSubsystem.readInputs();
        this.trampSubsystem.readInputs();
        this.wallClimbSubsystem.readInputs();

    }

    public void RunRobotPeriodicAfterCommandScheduler() {

        // This records outputs, e.g. to NetworkTables
        this.noteDetector.recordOutputs();
        this.canivoreSubsystem.recordOutputs();
        this.angleSubsystem.recordOutputs();
        this.elevatorSubsystem.recordOutputs();
        this.indexerSubsystem.recordOutputs();
        this.intakeSubsystem.recordOutputs();
        this.shooterSubsystem.recordOutputs();
        this.swerveSubsystem.recordOutputs();
        this.trampSubsystem.recordOutputs();
        this.wallClimbSubsystem.recordOutputs();

    }

    public void AutonomousInit() {
        this.swerveSubsystem.autonomousInit();
    }

    public void RunAutonomousPeriodic() {

    }

    public void TeleopInit() {
        this.swerveSubsystem.teleopInit();
    }

    public void RunTeleopPeriodic() {
        if(this.swerveSubsystem.IsInitialized()) {
            if(this.swerveSubsystem.GetDriveTowardsWallDuringClimb()) {
                var pitch_deg = this.gyro.getPitch_deg();
                if(pitch_deg > -5.0) {
                    this.swerveSubsystem.runSwerveRobotOrientedForwardSlow();
                }
                else {
                    this.swerveSubsystem.runSwerveRobotOrientedStop();
                }
            }
            else {
                this.swerveSubsystem.executeOperatorControl();
            }
        }
        else {
            this.swerveSubsystem.runSwerveRobotOrientedStop();
        }
    }

    public void DisabledInit() {
        this.xbox.setRumble(RumbleType.kLeftRumble, 0);
        this.xbox.setRumble(RumbleType.kRightRumble, 0);
    }

    public void RunDisabledPeriodic() {
        
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.get();
        //return this.autoCommand;
    }

    public double getTotalCurrent_A() {
        // Note: Intake, Shooter, Tramp and Wall Climb use NEO and/or Vortex and therefore don't report input current
      return this.angleSubsystem.getMotorSupplyCurrent()
        + this.elevatorSubsystem.getMotorSupplyCurrent()
        + this.indexerSubsystem.getMotorSupplyCurrent()
        + this.swerveSubsystem.GetDriveSupplyCurrent();
    }
    
}
