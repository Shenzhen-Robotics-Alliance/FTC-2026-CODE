package org.firstinspires.ftc.teamcode.commands.visionCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

public class SortCommand extends CommandBase{
    private VisionSubsystem visionSubsystem;

    public SortCommand(VisionSubsystem visionSubsystem){
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }
    public void initialize(){
        visionSubsystem.start();
    }
    public void execute(){
        visionSubsystem.periodic();
        if (visionSubsystem.getTargetID()!=21 || visionSubsystem.getTargetID()!=24) {
            final int Sorting_ID = visionSubsystem.getTargetID();
        }
    }
    public void end(boolean interrupted){

    }
    public boolean isFinished(){
        return false;
    }
}
