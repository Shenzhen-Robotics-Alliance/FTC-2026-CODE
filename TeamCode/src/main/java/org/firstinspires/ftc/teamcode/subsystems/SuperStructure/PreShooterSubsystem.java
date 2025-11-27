package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PreShooterSubsystem {
    public final Servo preShooter;

    public PreShooterSubsystem(HardwareMap hardwareMap){
        this.preShooter = hardwareMap.get(Servo.class,"PreShooter");
    }

    public Command setShootingAngle(){
        return new RunCommand(() ->
                preShooter.setPosition(0)
        );
     }

    public Command setStopPreShooter(){
        return new RunCommand(() ->
                preShooter.setPosition(0.5));
    }

    
}
