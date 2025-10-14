package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase{
    public final LinearMotion intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intake = new LinearMotion(
                "intake",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"IntakeMotor")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"Rot"),
                false,
                1600, // 1600/2000 = 80%
                0,
                0.02,
                0
        );
    }

    public void perioic(){
        intake.periodic();
    }


    public void setStopIntake(){
        intake.setMotorsStop();
    }

}