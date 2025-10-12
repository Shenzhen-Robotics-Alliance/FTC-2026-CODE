package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

public class AutoRotCommand extends CommandBase {
    private final RotSubsystem rotSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final double MAXIMUM_VELOCITY = 1500.0; //1500/2000
    private double CameraHorizontalPOV = 27.25;//54.5/2

    private double tx;
    private double ty;

    public AutoRotCommand(RotSubsystem rotSubsystem,VisionSubsystem visionSubsystem){
        this.rotSubsystem = rotSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(rotSubsystem,visionSubsystem);
    }
    @Override
    public void initialize(){
        visionSubsystem.start();
        rotSubsystem.setRotateStop();
    }
    @Override
    public void execute(){
        double targetVelocity = tx/CameraHorizontalPOV* MAXIMUM_VELOCITY;

        // 设置RotSubsystem的目标速度
        rotSubsystem.setRotateVelocity(targetVelocity);
    }

    public void end(boolean interrupted){
        rotSubsystem.rotateMotion.setMotorsStop();
    }

    public boolean isFinished(){
        return false;
    }

}
