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

    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                },
                new boolean[]{true},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor1"),
                true,
                3070,
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
        telemetry.addData("Output Power", "%.3f", shooter.getOutputPower());
        telemetry.addData("Encoder Pos", "%.0f", shooter.getPosition());
        telemetry.addData("===== SHOOTER STATUS =====", "");
        telemetry.addData("ready to short shoot?",isReadyToFixShortLaunch());
        telemetry.addData("ready to far shoot?",isReadyToFixFarLaunch());


    }


    // <Fixed Point shoot in both Short and Far Point>

    public Command shooterFixFarLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(0.8));
    }

    public boolean isReadyToFixFarLaunch(){
        return shooter.getCurrentVelocityRaw() > 200                ;
    }

    public boolean isReadyToFixShortLaunch(){
        return shooter.getCurrentVelocityRaw() > 1000;
    }

    public Command shooterFixShortLaunch(){
        return new RunCommand(() -> shooter.setTargetVelocity(0.5));
    }

    //<Shoot as the odometry>


    public Command setShootingMotorStop(){
        return new RunCommand(() -> shooter.setTargetVelocity(0));
    }


}
