package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;

public class OuttakeContinueCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public OuttakeContinueCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.intake.setMotorsStop();
    }

    @Override
    public void execute(){
        intakeSubsystem.periodic();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.intake.setMotorsStop();
    }
}
