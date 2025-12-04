package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;

import static org.firstinspires.ftc.teamcode.constants.SystemConstants.telemetry;
import static org.firstinspires.ftc.teamcode.constants.SystemConstants.ROT_INIT_POSITION;
public class AutoRotCommand extends CommandBase {
    private final RotSubsystem rotSubsystem;
    private final VisionSubsystem visionSubsystem;
    private double CameraHorizontalPOV = 30;//54.5/2

    private double tx;
    private double ty;
    private int BLUE_TARGET = 20;
    private int RED_TARGET = 24;
    private int TARGET_ID = 0;
    private int Current_ID = 0;
    private double Target_Distance = 0;


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
    }
    @Override
    public void execute() {
        visionSubsystem.periodic();
        tx = visionSubsystem.targetX;
        Target_Distance = visionSubsystem.Distance;
        Current_ID = visionSubsystem.CurrentID;

        telemetry.addData("==============Vision==================","");
        telemetry.addData("Target_Distance","%.3f",Target_Distance);
        telemetry.addData("Target_ID",this.TARGET_ID);
        telemetry.addData("Current_ID",this.Current_ID);

        double targetVelocity = (tx / CameraHorizontalPOV);
        if (Current_ID == TARGET_ID) {
            rotSubsystem.setRotateVelocity(
                    visionSubsystem.hasTarget ? targetVelocity * (Math.abs(tx) < 8 ? 1.5 : 0.9)
                            : 0
            );
        }
    }
    public void end(boolean interrupted){
        rotSubsystem.rotateMotion.setMotorsStop();
    }

    public boolean isFinished(){
        return false;
    }}