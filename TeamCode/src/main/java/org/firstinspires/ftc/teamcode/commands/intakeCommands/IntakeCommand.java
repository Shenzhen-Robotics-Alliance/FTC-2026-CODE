package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.PreShooterSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;
    private final PreShooterSubsystem preShooterSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem,PreShooterSubsystem preShooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.preShooterSubsystem = preShooterSubsystem;
    }

    public Command intakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //enable the intake servo
        sequence.addCommands(intakeSubsystem.setIntakeAngle()
                .alongWith(intakeSubsystem.enableIntakeMotor())
                .alongWith(preShooterSubsystem.setPreventAngle()));

        return sequence;
    }

    public Command outtakeContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //enable the servo
        sequence.addCommands(intakeSubsystem.setOuttakeAngle()
                .alongWith(intakeSubsystem.enableOuttakeMotor())
                .alongWith(preShooterSubsystem.setPreventAngle())
        );

        return sequence;
    }

    public Command stopIntake(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(intakeSubsystem);

        //stop both the servo and motor
        sequence.addCommands(intakeSubsystem.setStopAngle()
                .alongWith(intakeSubsystem.enableStopMotor())
                .alongWith(preShooterSubsystem.setStopPreShooter())
        );

        return sequence;
    }

}
