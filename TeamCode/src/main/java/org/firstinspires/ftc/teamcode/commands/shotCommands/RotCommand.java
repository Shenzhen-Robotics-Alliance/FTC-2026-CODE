package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;

public class RotCommand extends CommandBase{
    private RotSubsystem rotSubsystem;

    public RotCommand(RotSubsystem rotSubsystem){
        this.rotSubsystem = rotSubsystem;
        addRequirements(rotSubsystem);
    }

    public void initialize(){
        rotSubsystem.rotate.setMotorsStop();
    }

    public void execute(){
        super.execute();
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){
        rotSubsystem.rotate.setMotorsStop();
    }
}
