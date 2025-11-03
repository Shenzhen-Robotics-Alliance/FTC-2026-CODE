package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command intakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //init

        //enable the intake servo
        sequence.addCommands(intakeSubsystem.setIntakeAngle());

        //enable the intake motor
        sequence.addCommands(intakeSubsystem.enableIntakeMotor());

        return sequence;
    }

    public Command outtakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //enable the servo
        sequence.addCommands(intakeSubsystem.setOuttakeAngle());

        //enable the motor
        sequence.addCommands(intakeSubsystem.enableOuttakeMotor());

        return sequence;
    }

    public Command stopIntake(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //stop both the servo and motor
        sequence.addCommands(intakeSubsystem.setStopAngle()
                .alongWith(intakeSubsystem.enableStopMotor()));

        return sequence;
    }


}
