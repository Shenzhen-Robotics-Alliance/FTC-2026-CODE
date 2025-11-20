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
    public final Servo intakeServo;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intake = new LinearMotion(
                "intake",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"IntakeMotor")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"IntakeMotor"),
                false,
                2000,
                5,
                0,
                0,
                0
        );
        this.intakeServo = hardwareMap.get(Servo.class,"IntakeServo");
    }

    public void periodic(){
        intake.periodic();
    }

    public void setStopIntake(){
        intake.setMotorsStop();
    }

    //correct it according to the real situation

    public Command setIntakeAngle(){
        return new RunCommand(() -> intakeServo.setPosition(0));
    }

    public Command setOuttakeAngle(){
        return new RunCommand(() -> intakeServo.setPosition(1));
    }

    public Command setStopAngle(){
       return new InstantCommand(()->intakeServo.setPosition(0.5));
    }

    public Command enableIntakeMotor(){
        return new RunCommand(() -> intake.setTargetVelocity(-0.8));
    }

    public Command enableOuttakeMotor(){
        return new RunCommand(() -> intake.setTargetVelocity(0.8));
    }

    public Command enableStopMotor(){
        return new RunCommand(() -> intake.setTargetVelocity(0));
    }
}