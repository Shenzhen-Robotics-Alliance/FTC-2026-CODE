package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryTransmission;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Shooter Monitor", group = "Debug")
public class MonitoringShooterMotor extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private VoltageSensor voltageSensor;
    private FtcDashboard dashboard;
    private ElapsedTime timer;

    // 配置参数（可调）
    private static final String MOTOR_NAME = "ShooterMotor1";
    private static final double TARGET_POWER = 0.8;
    private static final double RAMP_UP_STEP = 0.05;     // 每次增加功率
    private static final long RAMP_UP_DELAY = 200;       // 每次增加间隔 (ms)
    private static final double OVERCURRENT_THRESHOLD = 25.0; // 过流保护 (A)
    private static final double LOW_VOLTAGE_THRESHOLD = 9.5;  // 低电压报警 (V)

    @Override
    public void runOpMode() {
        // 初始化硬件
        shooterMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();

        // 设置电机模式
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 多重遥测（Driver Station + Dashboard）
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Dashboard 设置
        dashboard.setTelemetryTransmissionInterval(100); // 10Hz 刷新

        telemetry.addLine("Shooter Monitor Initialized");
        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.addData("Target Power", TARGET_POWER);
        telemetry.update();

        waitForStart();
        timer.reset();

        // 渐进加速
        rampUpPower();

        // 主循环
        while (opModeIsActive()) {
            shooterMotor.setPower(0.8);
            double current = shooterMotor.getCurrent(CurrentUnit.AMPS);
            double voltage = voltageSensor.getVoltage();
            double power = shooterMotor.getPower();
            double runtime = timer.seconds();

            // 过流保护
            if (current > OVERCURRENT_THRESHOLD) {
                shooterMotor.setPower(power * 0.7); // 降 30%
                telemetry.addData("ALERT", "OVERCURRENT! Power reduced");
            }

            // 低电压报警
            if (voltage < LOW_VOLTAGE_THRESHOLD) {
                telemetry.addData("WARNING", "LOW VOLTAGE! Check battery/connection");
            }

            // 发送到 Dashboard（带时间戳）
            telemetry.addData("Runtime (s)", "%.2f", runtime);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Current (A)", "%.2f", current);
            telemetry.addData("Voltage (V)", "%.2f", voltage);

            // 图表数据（Dashboard 会自动绘图）
            dashboard.getTelemetry().addData("Current", current);
            dashboard.getTelemetry().addData("Voltage", voltage);
            dashboard.getTelemetry().addData("Power", power);

            telemetry.update();
            sleep(100); // 10Hz
        }

        // 停止
        shooterMotor.setPower(0);
        telemetry.addLine("Test Complete. Data saved to Dashboard.");
        telemetry.update();
    }

    private void rampUpPower() {
        telemetry.addLine("Ramping up power...");
        telemetry.update();

        for (double p = 0.1; p <= TARGET_POWER; p += RAMP_UP_STEP) {
            shooterMotor.setPower(p);
            telemetry.addData("Ramping", "%.3f", p);
            telemetry.update();
            sleep(RAMP_UP_DELAY);
        }

        shooterMotor.setPower(TARGET_POWER);
        telemetry.addData("Target Power Reached", TARGET_POWER);
        telemetry.update();
    }
}