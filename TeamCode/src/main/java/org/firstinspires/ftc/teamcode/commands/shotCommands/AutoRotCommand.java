package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

public class AutoRotCommand extends CommandBase {
    private final RotSubsystem rotSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AutoRotCommand(RotSubsystem rotSubsystem,VisionSubsystem visionSubsystem){
        this.rotSubsystem = rotSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(rotSubsystem,visionSubsystem);
    }

}
