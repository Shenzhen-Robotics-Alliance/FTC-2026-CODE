package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.ShooterSubsystem;

public class OuttakeContinueCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public OuttakeContinueCommand(IntakeSubsystem intakeSubsystem,ShooterSubsystem shooterSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.intake.setMotorsStop();
        shooterSubsystem.shooter.setMotorsStop();
    }

    @Override
    public void execute(){
        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.intake.setMotorsStop();
        shooterSubsystem.shooter.setMotorsStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
