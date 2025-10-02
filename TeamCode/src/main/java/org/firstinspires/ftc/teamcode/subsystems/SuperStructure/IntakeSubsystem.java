package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase{
    public final LinearMotion intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intake = new LinearMotion(
                "intake",
                new DcMotor[]{hardwareMap.get(DcMotor.class,"IntakeMotor")},
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

    public void setStopIntake(){
        intake.setMotorsStop();
    }

}