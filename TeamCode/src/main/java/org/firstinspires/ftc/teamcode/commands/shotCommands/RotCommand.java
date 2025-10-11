package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;

import java.util.function.Supplier;

public class RotCommand extends CommandBase{
    private RotSubsystem rotSubsystem;
    private Supplier<Double> joystickSupplier;

    public RotCommand(RotSubsystem rotSubsystem, Supplier<Double> joystickSupplier){
        this.rotSubsystem = rotSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(rotSubsystem);
    }

    /**
    public void initialize(){
        rotSubsystem.rotate.setMotorsStop();
    }
    */

    public void execute(){
        double joystickValue = joystickSupplier.get();

        if(Math.abs(joystickValue) > 0.15) {
            double currentPos = rotSubsystem.rotate.getCurrentSetPoint();
            double increment = joystickValue * 20;
            rotSubsystem.rotate.goToPosition(currentPos + increment);
            rotSubsystem.rotate.periodic();
        }else{
            rotSubsystem.rotate.setMotorsStop();
        }
    }

    public void end(boolean interrupted){
        rotSubsystem.rotate.setMotorsStop();
    }

    public boolean isFinished(){
        return false;
    }

}
