package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import static org.firstinspires.ftc.teamcode.constants.SystemConstants.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;

public class ShooterSubsystem extends SubsystemBase {
    public final LinearMotion shooter;

    public final Servo shootServo;


    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
//                        hardwareMap.get(DcMotorEx.class,"ShooterMotor2")
                },
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                false,
                2600,
                0.5,
                0,
                0,
                0
        );
        this.shootServo = hardwareMap.get(Servo.class,"shootServo");
    }

    public void periodic() {
        shooter.periodic();
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Target Vel", "%.0f RPM", shooter.getTargetVelocity()); //* 2000
        telemetry.addData("Current Vel", "%.0f RPM", shooter.getCurrentVelocityRaw());
        telemetry.addData("Vel Error", "%.0f RPM", shooter.getVelocityError()); //* 2000
        telemetry.addData("Output Power", "%.3f", shooter.getOutputPower());
        telemetry.addData("Encoder Pos", "%.0f", shooter.getPosition());
    }
    public void setShooterStop(){
        shooter.setMotorsStop();
    }

    public void setShootingVelocity(double velocity){
        shooter.setTargetVelocity(velocity);
    }

    public Command shooterStop(){
        return new InstantCommand(() -> shooter.setMotorsStop());
    }

    public Command shooterFarLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(0.8));
    }

    public boolean isReadyToFarLaunch(){
        return shooter.getCurrentVelocity() > 0.9 ? true : false;
    }


    public boolean isReadyToShortLaunch(){
        return shooter.getCurrentVelocity() > 0.6 ? true : false;

    }

    public Command setFarShootingAngle(){
        return new InstantCommand(() -> shootServo.setPosition(1));
    }

    public Command setHoldBallAngle(){
        return new InstantCommand(() -> shootServo.setPosition(0.5));
    }


    public Command shooterShortLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(0.3));
    }


    public Command setShortShootingAngle(){
        return new InstantCommand(() -> shootServo.setPosition(1));
    }
//    public void setShooterVelocityRPM(double rpm) {
//        shooter.setTargetVelocity(rpm / 2000); // 1500 / 2600 = 0.577
//    }
}
