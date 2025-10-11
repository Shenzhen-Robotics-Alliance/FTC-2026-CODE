package org.firstinspires.ftc.teamcode.commands.visionCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

public class FollowCommand extends CommandBase{
    private VisionSubsystem visionSubsystem;

    public FollowCommand(VisionSubsystem visionSubsystem){
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    public void initialize(){
        visionSubsystem.start();
    }
    public void execute(){
        visionSubsystem.periodic();
    }
    public void end(boolean interrupted){
        visionSubsystem.stop();
    }
    public boolean isFinished(){
        return false;
    }

}
