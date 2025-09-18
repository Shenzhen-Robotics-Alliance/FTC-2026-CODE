package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Optional;

public class IntakeSubsystem extends SubsystemBase{
    private final LinearMotion intake;

    public IntakeSubsystem(HardwareMap hardwareMap){
        this.intake = new LinearMotion(
                "IntakeMotor",
                new DcMotor[]{hardwareMap.get(DcMotor.class,"intake")},
                new boolean[]{true},
                hardwareMap.get(DcMotorEx.class,"intake"),
                true,
                Optional.empty(),
                1800,
                0,
                0,
                0,
                0);
    }

    public void periodic(){

    }

}