package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;
//2,=========shooterSubsystem==========
import static org.firstinspires.ftc.teamcode.constants.SystemConstants.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem extends SubsystemBase {
    public final LinearMotion shooter;

    public final Servo shootServo1;

  //  public final Servo shootServo2;


    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
//                        hardwareMap.get(DcMotorEx.class,"ShooterMotor2")
                },
                new boolean[]{true},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                true,
                2500,
                3,
                0,
                0.003,
                0
        );
        this.shootServo1 = hardwareMap.get(Servo.class,"shootServo1");
      //  this.shootServo2 = hardwareMap.get(Servo.class, "shootServo2");
    }

    public void periodic() {
        shooter.periodic();
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Target Vel", "%.0f RPM", shooter.getTargetVelocity()); //* 2000
        telemetry.addData("Current Vel", "%.0f RPM", shooter.getCurrentVelocityRaw());
        telemetry.addData("Vel Error", "%.0f RPM", shooter.getVelocityError()); //* 2000
        telemetry.addData("Output Power", "%.3f", shooter.getOutputPower());
        telemetry.addData("Encoder Pos", "%.0f", shooter.getPosition());
        telemetry.addData("===== SHOOTER STATUS =====", "");
        telemetry.addData("ready to short shoot?",isReadyToShortLaunch());
        telemetry.addData("ready to far shoot?",isReadyToFarLaunch());
    }
    public void setShooterStop(){
        shooter.setMotorsStop();
    }

    public void setShootingVelocity(double velocity){
        shooter.setTargetVelocity(velocity);
    }

    public Command shooterStop(){
        return new InstantCommand(shooter::setMotorsStop);
    }

    public Command shooterFarLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(0.8));
    }

    public boolean isReadyToFarLaunch(){
        return shooter.getCurrentVelocity() > 0.75;
    }

    public boolean isReadyToShortLaunch(){
        return shooter.getCurrentVelocity() > 0.4;

    }

    public Command setFarShootingAngle(){
        return new InstantCommand(() ->
            shootServo1.setPosition(0));
        }

    public Command setHoldBallAngle(){
        return new InstantCommand(() ->
            shootServo1.setPosition(0.5)
        );
    }

    public Command shooterShortLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(0.5));
    }

    public Command setShortShootingAngle(){
        return new InstantCommand(() ->
            shootServo1.setPosition(0)
        );
    }

    public Command setShootingMotorStop(){
        return new RunCommand(() -> shooter.setTargetVelocity(0));
    }
}
