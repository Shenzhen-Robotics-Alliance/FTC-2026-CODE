package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.PreShooterSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command intakeContinuously() {
        return new StartEndCommand(
                () -> intakeSubsystem.enableIntakeMotor().schedule(),
                () -> intakeSubsystem.enableStopMotor().schedule(),
                intakeSubsystem
        );
    }
    public Command outtakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        sequence.addCommands(intakeSubsystem.enableOuttakeMotor());

        return sequence;
    }

    public Command stopIntake(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        sequence.addCommands(intakeSubsystem.enableStopMotor());

        return sequence;
    }

}
