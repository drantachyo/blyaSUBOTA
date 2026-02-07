package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx motor;

    // Настройки мощностей
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;


    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);
        // FLOAT лучше для интейка, чтобы он плавно останавливался и не бил по механике
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake() {
        motor.setPower(INTAKE_POWER);
    }

    public void outtake() {
        motor.setPower(OUTTAKE_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }
}