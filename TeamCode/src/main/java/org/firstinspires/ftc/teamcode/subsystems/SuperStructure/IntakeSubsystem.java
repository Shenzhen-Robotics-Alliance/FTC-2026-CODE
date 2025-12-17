package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase{
    public final LinearMotion intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intake = new LinearMotion(
                "intake",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"IntakeMotor")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"IntakeMotor"),
                false,
                2000,
                5,  //0
                0,
                0,
                0
        );
    }

    public void periodic(){
        intake.periodic();
    }

    public Command enableIntakeMotor(){
        return new InstantCommand(() -> intake.setTargetVelocity(1));
    }

    public Command enableOuttakeMotor(){
        return new InstantCommand(() -> intake.setTargetVelocity(-1));
    }

    public Command enableStopMotor(){
        return new InstantCommand(() -> intake.setTargetVelocity(0));
    }
}