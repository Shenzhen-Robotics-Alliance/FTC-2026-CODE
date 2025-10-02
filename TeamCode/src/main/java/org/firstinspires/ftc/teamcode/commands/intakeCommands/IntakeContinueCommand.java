package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;

public class IntakeContinueCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public IntakeContinueCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intake.setMotorsStop();
    }

    public void execute(){
        intakeSubsystem.periodic();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){
        intakeSubsystem.intake.setMotorsStop();
    }
}
