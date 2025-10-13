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
    private static double kP = 5;
    private static double kI = 1;
    private static double kD = 0.2;
    private static double kF = 0.0;
    private static double target = 0;
    private PIDFController pidfController = new PIDFController(kP, kI, kD, kF);

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
        tx = visionSubsystem.getTargetX();
        if (tx>=1) {
            double targetVelocity = (tx / CameraHorizontalPOV);
            double power = -pidfController.calculate(targetVelocity,target);


            // Set targetVelocity of the rotSubsystem
            rotSubsystem.setManualRotPower(power);
        }
        else {
            rotSubsystem.setRotateVelocity(0);
        }

        }

    public void end(boolean interrupted){
        rotSubsystem.rotateMotion.setMotorsStop();
    }

    public boolean isFinished(){
        return false;
    }

}
