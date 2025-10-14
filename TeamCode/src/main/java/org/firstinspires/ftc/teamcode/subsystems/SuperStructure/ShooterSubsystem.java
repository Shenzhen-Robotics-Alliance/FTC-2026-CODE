package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    public final LinearMotion shooter;

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
                1800, // 1800/2000
                0,
                0.01,
                0
        );
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

    public double getCurrentRotateVelocity() {
        return shooter.getCurrentVelocity();
    }

    public double getTargetRotateVelocity() {
        return shooter.getCurrentSetPoint();
    }

}
