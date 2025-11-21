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

    @Override
    public void initialize(){
        intakeSubsystem.setStopIntake();
        intakeSubsystem.setStopAngle();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setStopIntake();
        intakeSubsystem.setStopAngle();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public Command intakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //init

        //enable the intake servo
        sequence.addCommands(intakeSubsystem.setIntakeAngle()
                .alongWith(intakeSubsystem.enableIntakeMotor()));

        return sequence;
    }

    public Command outtakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //enable the servo
        sequence.addCommands(intakeSubsystem.setOuttakeAngle()
                .alongWith(intakeSubsystem.enableOuttakeMotor())
        );

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
