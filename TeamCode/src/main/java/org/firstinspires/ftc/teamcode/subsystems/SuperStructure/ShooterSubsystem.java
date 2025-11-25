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

    private final double MOTOR_CPR = 103.6; //counts per revolution
    private final double MAX_TICKS_PER_SEC = 3073; //Ticks/Sec: (1780 * 103.6) / 60 ≈ 3073
    private final double TOLERANCE_RPM = 50.0;
    private final double TOLERANCE_TICKS = (TOLERANCE_RPM * MOTOR_CPR) / 60.0;



    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                },
                new boolean[]{true},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                true,
                MAX_TICKS_PER_SEC,
                0.8,
                0,
                0.003,
                0
        );
    }

    public void periodic() {
        shooter.periodic();
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Target Vel", "%.0f RPM", shooter.getTargetVelocity()); //* 2000
        telemetry.addData("Current Vel", "%.0f RPM", shooter.getCurrentVelocityRaw());
        telemetry.addData("Vel Error", "%.0f RPM", shooter.getVelocityError()); //* 2000
        telemetry.addData("Encoder Pos", "%.0f", shooter.getPosition());
        telemetry.addData("===== SHOOTER STATUS =====", "");
        telemetry.addData("ready to short shoot?",isReadyToFixShortLaunch());
        telemetry.addData("ready to far shoot?",isReadyToFixFarLaunch());

    }

    //<General Functions for the motor control>
    public void setTargetRPM(double rpm) {
        //limit control motor
        double clampedRPM = Math.min(rpm, 1780.0);

        //Translate rpm to Ticks/Sec
        double targetTPS = (clampedRPM * MOTOR_CPR) / 60.0;

        //normalized to (0.0 - 1.0)
        double normalized = targetTPS / MAX_TICKS_PER_SEC;

        shooter.setTargetVelocity(Math.max(0, normalized));
    }

    public boolean isAtTargetSpeed() {
        // get Ticks/Sec
        double currentTPS = shooter.getCurrentVelocityRaw();

        double targetTPS = shooter.getTargetVelocity() * MAX_TICKS_PER_SEC;

        return Math.abs(currentTPS - targetTPS) < TOLERANCE_TICKS;
    }



    //<Fixed Point shoot in both Short and Far Point>
    public Command shooterFixFarLaunch(){
        return new RunCommand(() -> setTargetRPM(1500));
    }


    public Command shooterFixShortLaunch(){
        return new RunCommand(() -> setTargetRPM(1200));
    }

    public boolean isReadyToFixFarLaunch(){
        return isAtTargetSpeed();
    }

    public boolean isReadyToFixShortLaunch(){
        return isAtTargetSpeed();
    }



    //<Shoot as the odometry>



    public Command setShootingMotorStop(){
        return new RunCommand(() -> shooter.setTargetVelocity(0));
    }


}
