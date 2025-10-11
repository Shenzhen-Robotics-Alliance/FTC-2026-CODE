package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotSubsystem extends SubsystemBase {
    public final LinearMotion rotate;

    public RotSubsystem(HardwareMap hardwareMap){
        this.rotate = new LinearMotion(
                "Rotate",
                new DcMotor[]{hardwareMap.get(DcMotor.class,"Rot")},
                new boolean[]{false},
                hardwareMap.get(DcMotor.class,"Rot"),
                false,
                1600, // 1600/2000 = 80%
                0,
                0.02,
                0,
                0.01
        );
    }

    public void periodic(){
        rotate.periodic();
    }


    public void setRotateStop(){
        rotate.setMotorsStop();
    }
}
