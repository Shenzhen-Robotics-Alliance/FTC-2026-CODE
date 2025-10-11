package org.firstinspires.ftc.teamcode.commands.visionCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

public class VisionCommands extends CommandBase{
    private VisionSubsystem visionSubsystem;

    public VisionCommands(VisionSubsystem visionSubsystem){
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
