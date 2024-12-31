package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.angle.AngleSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;
import frc.robot.subsystems.wallclimb.WallClimbSubsystem;

// Transfers the note from the indexer to the tramp scorer
public class TransferCommand extends Command {
    
    private final IndexerSubsystem indexerSubsystem;
    private final AngleSubsystem angleSubsystem;
    private final WallClimbSubsystem wallclimbSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final TrampSubsystem trampSubsystem;
    private boolean atLowAngle = false;
    private boolean atSpeed = false;
    private boolean lowSpeed = false;
    private boolean noteSeen = false;
    private boolean isFinished = false;

    public TransferCommand(
            IndexerSubsystem indexerSubsystem,
            AngleSubsystem angleSubsystem,
            WallClimbSubsystem wallclimbSubsystem,
            IntakeSubsystem intakeSubsystem,
            TrampSubsystem trampSubsystem){

        this.indexerSubsystem = indexerSubsystem;
        this.angleSubsystem = angleSubsystem;
        this.wallclimbSubsystem = wallclimbSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.trampSubsystem = trampSubsystem;

        addRequirements(indexerSubsystem);
        addRequirements(angleSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(trampSubsystem);
        addRequirements(wallclimbSubsystem);
    }

    @Override
    public void initialize() {
        this.trampSubsystem.SetLowCurrentLimit();
        this.trampSubsystem.SetDutyCycle(0.5);
        this.wallclimbSubsystem.RunForward(1500);
        this.indexerSubsystem.SetModeStop();
        this.intakeSubsystem.TransferNote();
        this.angleSubsystem.GoToLowTransferPosition();
        this.atLowAngle = false;
        this.atSpeed = false;
        this.lowSpeed = false;
        this.noteSeen = false;
        this.isFinished = false;
        this.indexerSubsystem.SetNotePresent(false);
    }

    @Override
    public void execute() {

        if(!this.atLowAngle && this.angleSubsystem.AtLowTransferPosition()) {
            this.atLowAngle = true;
            this.indexerSubsystem.SetModeTransfer();
            this.angleSubsystem.GoToHighTransferPosition();
        }

        if(!this.atSpeed && this.trampSubsystem.GetVelocityRpm() > 1800) {
            this.atSpeed = true;
        }

        if(this.atSpeed && !this.lowSpeed && this.trampSubsystem.GetVelocityRpm() < 1200) {
            this.lowSpeed = true;
        }
        
        if(this.lowSpeed && !this.noteSeen) {
            this.trampSubsystem.ZeroPosition();
            this.noteSeen = true;
        }

        if(this.noteSeen && !this.isFinished && this.trampSubsystem.GetPosition_revs() > 0.5) {
            this.isFinished = true;
        }
    }

     @Override
     public boolean isFinished() {
         return this.isFinished;
     }

     @Override
     public void end(boolean interrupted) {
         this.trampSubsystem.Stop();
         this.wallclimbSubsystem.Stop();
         this.intakeSubsystem.Stop();
         this.indexerSubsystem.SetModeDefault();
         if (!interrupted){
            this.trampSubsystem.SetNotePresent(true);
         }
     }


}
