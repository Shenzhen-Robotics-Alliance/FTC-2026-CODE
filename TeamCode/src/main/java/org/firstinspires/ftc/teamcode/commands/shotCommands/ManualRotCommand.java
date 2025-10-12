package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;

import java.util.function.Supplier;

public class ManualRotCommand extends CommandBase{
    private final RotSubsystem rotSubsystem;
    private final Supplier<Double> joystickSupplier;
    private final double MAXIMUM_VELOCITY = 1500; //1500/2000

    private final double MANUAL_DEADBAND = 0.05;

    public ManualRotCommand(RotSubsystem rotSubsystem, Supplier<Double> joystickSupplier){
        this.rotSubsystem = rotSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(rotSubsystem);
    }


    public void initialize(){
        rotSubsystem.setRotateStop();
    }

    @Override
    public void execute(){
        double joystickValue = joystickSupplier.get();

        //apply the deadBand
        if (Math.abs(joystickValue) < MANUAL_DEADBAND) {
            joystickValue = 0;
        }

        double targetVelocity = joystickValue * MAXIMUM_VELOCITY; // 假设负摇杆Y轴对应正速度

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
