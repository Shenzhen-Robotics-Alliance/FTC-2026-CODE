package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    public final LinearMotion shooter;

   // public final Servo shootServo;

    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor2")
                },
                new boolean[]{false,false},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                false,
                2000,
                0.2,
                0.01,
                0.2,
                0
        );
     //    this.shootServo = hardwareMap.get(Servo.class,"shootServo");

    }

    public void periodic(){
        shooter.periodic();
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

    /**

    public Command shooterFarLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(2000));
    }

    public boolean isReadyToFarLaunch(){
        if(shooter.getCurrentVelocity() > 0.9){
            return true;
        }else{
            return false;
        }
    }

    public boolean isReadyToShortLaunch(){
        if(shooter.getCurrentVelocity() > 0.7){
            return true;
        }else{
            return false;
        }
    }


    public Command setFarShootingAngle(){
          return new InstantCommand(() -> shootServo.setPosition(0.5));
    }
    public Command setHoldBallAngle(){
        return new InstantCommand(() -> shootServo.setPosition(0));
    }

    public Command shooterShortLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(1500));
    }

    public Command setShortShootingAngle(){
        return new InstantCommand(() -> shootServo.setPosition(0.5));
    }

*/
}
