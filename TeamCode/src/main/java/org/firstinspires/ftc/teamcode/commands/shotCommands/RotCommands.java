package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;

public class RotCommands extends CommandBase{
    private RotSubsystem rotSubsystem;
    public RotCommands(RotSubsystem rotSubsystem){
        this.rotSubsystem = rotSubsystem;
        addRequirements(rotSubsystem);
    }

    public void initialize(){
        rotSubsystem.setRotStop();
    }

    public void execute(){
        rotSubsystem.periodic();
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){rotSubsystem.setRotStop();}
}
