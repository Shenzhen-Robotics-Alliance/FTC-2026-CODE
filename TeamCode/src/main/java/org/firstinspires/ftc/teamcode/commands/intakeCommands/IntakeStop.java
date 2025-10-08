package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;

public class IntakeStop extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public IntakeStop(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.intake.setMotorsStop();
    }

    public void execute(){
        intakeSubsystem.intake.setMotorsStop();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.intake.setMotorsStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
