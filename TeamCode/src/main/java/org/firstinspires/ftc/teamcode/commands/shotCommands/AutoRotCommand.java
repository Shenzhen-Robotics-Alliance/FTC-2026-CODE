package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import com.arcrobotics.ftclib.command.ConditionalCommand;
public class AutoRotCommand extends CommandBase {
    private final RotSubsystem rotSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final double MAXIMUM_VELOCITY = 2000; //1500/2000
    private double CameraHorizontalPOV = 20;//54.5/2

    private double tx;
    private double ty;
    private int BLUE_TARGET = 20;
    private int RED_TARGET = 24;
    private int TARGET_ID = 0;


    public AutoRotCommand(RotSubsystem rotSubsystem, VisionSubsystem visionSubsystem,AllianceSide side){
        this.rotSubsystem = rotSubsystem;
        this.visionSubsystem = visionSubsystem;
        if (side == AllianceSide.BLUE) {
            TARGET_ID = BLUE_TARGET;
        } else if (side == AllianceSide.RED) {
            TARGET_ID = RED_TARGET;
        }

        addRequirements(rotSubsystem,visionSubsystem);
    }
    @Override
    public void initialize(){
        visionSubsystem.start();
        rotSubsystem.setRotateStop();

    }
    @Override
    public void execute() {
        visionSubsystem.periodic();
        if (visionSubsystem.hasTarget && visionSubsystem.getTargetID() == TARGET_ID) {
            tx = visionSubsystem.getTargetX();
            double targetVelocity = (tx / CameraHorizontalPOV);

//            double pos = rotSubsystem.getCurrentPosition();

//            if (Math.abs(pos) > ROT_SOFT_LIMIT)
//            {
//                if (pos < -ROT_SOFT_LIMIT && targetVelocity > 0)
//                {
//                    rotSubsystem.setRotateVelocity(0);
//                } else if (pos > ROT_SOFT_LIMIT && targetVelocity < 0)
//                {
//                    rotSubsystem.setRotateVelocity(0);
//                } else {
//                    rotSubsystem.setRotateVelocity(
//                            pos > 0 ? -1 : 1
//                    );
//
//                }
//            };
            // Set targetVelocity of the rotSubsystem
            rotSubsystem.setRotateVelocity(targetVelocity);
        }
        else {
            rotSubsystem.setRotateStop();
        }
    }
    public void end(boolean interrupted){
        rotSubsystem.rotateMotion.setMotorsStop();
    }

    public boolean isFinished(){
        return false;
    }}