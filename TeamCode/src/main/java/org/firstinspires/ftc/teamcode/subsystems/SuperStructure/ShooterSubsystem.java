package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    public final LinearMotion shooter;

    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "intake",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
//                              hardwareMap.get(DcMotor.class,"ShooterMotor2")
                },
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                false,
                1600, // 1600/2000 = 80%
                0,
                0.02,
                0,
                0.01
        );
    }

    public void periodic(){
        shooter.periodic();
    }

    public void setShooterStop(){
        shooter.setMotorsStop();
    }

}
