package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotSubsystem extends SubsystemBase {
    public final LinearMotion rotateMotion;

    public RotSubsystem(HardwareMap hardwareMap){
        this.rotateMotion = new LinearMotion(
                "Rotate",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"Rot")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"Rot"),
                false,
                2000,
                0.25,
                0,
                0.006,
                0
        );
    }

    public void periodic(){
        rotateMotion.periodic();
    }

    public void setRotateStop(){
        rotateMotion.setMotorsStop();
        rotateMotion.resetController();
    }

    public void setRotateVelocity(double velocity) {
        rotateMotion.setTargetVelocity(velocity);
    }

    public void setManualRotPower(double power){
        rotateMotion.runPower(power);
    }

    public double getCurrentRotateVelocity() {
        return rotateMotion.getCurrentVelocity();
    }

    public double getTargetRotateVelocity() {
        return rotateMotion.getCurrentSetPoint();
    }

}
