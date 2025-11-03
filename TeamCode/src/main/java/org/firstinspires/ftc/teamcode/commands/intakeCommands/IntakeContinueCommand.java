/**
package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;

import java.util.function.Supplier;

public class IntakeContinueCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private Supplier<Double> button;

    public IntakeContinueCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intake.setMotorsStop();
    }

    /**
    public void execute(){

        double buttonValue = button.get();

        if(Math.abs(buttonValue) > 0.15) {
            double currentPos = intakeSubsystem.intake.getCurrentSetPoint();
            double increment = buttonValue * 15;
            intakeSubsystem.setIntakeAngle();
            intakeSubsystem.intake.goToPosition(currentPos + increment);
            intakeSubsystem.intake.periodic();
        }else{
            intakeSubsystem.setStopAngle();
            intakeSubsystem.intake.setMotorsStop();
        }
        intakeSubsystem.periodic();
    }


    public void end(boolean interrupted){
        intakeSubsystem.intake.setMotorsStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
*/