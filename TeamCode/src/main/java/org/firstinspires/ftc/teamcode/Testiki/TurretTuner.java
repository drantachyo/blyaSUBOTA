package org.firstinspires.ftc.teamcode.Testiki;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "🔧 TURRET TUNER", group = "Tuning")
public class TurretTuner extends OpMode {

    private DcMotorEx turret;

    @Override
    public void init() {
        // Инициализация мотора (как в SimpleTurret)
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        // ВАЖНО: Если в SimpleTurret стоит REVERSE, тут тоже ставим REVERSE
        turret.setDirection(DcMotor.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Тормоз, чтобы не уезжала

        telemetry.addLine("READY. Use Left Stick Y to rotate.");
        telemetry.addLine("Press A to RESET encoders.");
    }

    @Override
    public void loop() {
        // Управление: Левый стик вверх/вниз (медленно, для точности)
        double power = -gamepad1.left_stick_y * 0.3;
        turret.setPower(power);

        // Сброс энкодера кнопкой A
        if (gamepad1.a) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        int currentTicks = turret.getCurrentPosition();

        telemetry.addData("--- DATA ---", "");
        telemetry.addData("TICKS", currentTicks);

        // Подсказка для расчета (если повернул на 90 градусов)
        if (currentTicks != 0) {
            double calculatedTPR = (double) currentTicks / (Math.PI / 2.0);
            telemetry.addData("If 90 deg now -> TPR =", "%.2f", calculatedTPR);
        }

        telemetry.update();
    }
}