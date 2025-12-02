package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PreShooterSubsystem {
    public final Servo preShooter1;
    public final Servo preShooter2;

    public PreShooterSubsystem(HardwareMap hardwareMap){
        this.preShooter1 = hardwareMap.get(Servo.class,"PreShooter1");
        this.preShooter2 = hardwareMap.get(Servo.class,"PreShooter2");
    }

    public Command setShootingAngle(){
        return new InstantCommand(() -> {
            preShooter1.setPosition(0.59);
            preShooter2.setPosition(0.65);
        });
     }

    public Command setStopPreShooter(){
        return new InstantCommand(() -> {
            preShooter1.setPosition(0.62);
            preShooter2.setPosition(0.62);
        });
    }

    
}
