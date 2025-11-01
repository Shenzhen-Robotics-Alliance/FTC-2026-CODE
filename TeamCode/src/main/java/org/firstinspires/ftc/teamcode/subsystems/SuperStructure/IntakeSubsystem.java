package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase{
    public final LinearMotion intake;
    public final Servo intakeServo;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intake = new LinearMotion(
                "intake",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"IntakeMotor")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"Rot"),
                false,
                2000,
                0,
                0.02,
                0,
                0
        );
        this.intakeServo = hardwareMap.get(Servo.class,"intakeServo");
    }

    public void periodic(){
        intake.periodic();
    }

    public void setStopIntake(){
        intake.setMotorsStop();
    }

    //correct it according to the real situation
    public void setIntakeAngle(){
      intakeServo.setPosition(0);
    }

    public void setOuttakeAngle(){
       intakeServo.setPosition(1);
    }

    public void setStopAngle(){
        intakeServo.setPosition(0.5);
    }

}