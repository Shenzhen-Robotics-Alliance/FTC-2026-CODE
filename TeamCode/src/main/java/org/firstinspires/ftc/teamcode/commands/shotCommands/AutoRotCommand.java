package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
public class AutoRotCommand extends CommandBase {
    private final RotSubsystem rotSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final double MAXIMUM_VELOCITY = 1200.0; //1500/2000
    private double CameraHorizontalPOV = 20;//54.5/2

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
        visionSubsystem.periodic();
        tx = visionSubsystem.getTargetX();
            double targetVelocity = (tx);
            double power = visionSubsystem.calculatePIDOutput(targetVelocity);


            // Set targetVelocity of the rotSubsystem
            rotSubsystem.setManualRotPower(power);

        }

    public void end(boolean interrupted){
        rotSubsystem.rotateMotion.setMotorsStop();
    }

    public boolean isFinished(){
        return false;
    }

}
