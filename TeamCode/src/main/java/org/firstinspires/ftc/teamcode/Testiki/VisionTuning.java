package org.firstinspires.ftc.teamcode.Testiki;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.math.MathUtils;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "👁️ VISION PID TUNING", group = "Tuning")
public class VisionTuning extends OpMode {

    private SimpleTurret turret;
    private Vision vision;

    // Какой тег ищем (меняй в панели: 24-Red Basket, 20-Blue Basket)
    public static int TARGET_TAG_ID = 23;

    private boolean cameraSetup = false;

    @Override
    public void init() {
        turret = new SimpleTurret(hardwareMap);
        vision = new Vision(hardwareMap);

        telemetry.addLine("READY TO TUNE VISION PID.");
        telemetry.addLine("1. Open Dashboard/Panel");
        telemetry.addLine("2. Find SimpleTurret -> VIS_P, VIS_D");
        telemetry.addLine("3. Shake robot gently to test stability");
    }

    @Override
    public void init_loop() {
        // Применяем настройки камеры для боя (низкая выдержка)
        if (!cameraSetup) {
            cameraSetup = vision.applyCombatSettings();
            if (cameraSetup) telemetry.addLine("✅ Camera Configured (2ms)");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Включаем трекинг (координаты 0,0 не важны, так как мы тестим только VIS PID)
        // Мы передаем pose = null, чтобы одометрия не вмешивалась
        turret.track(TARGET_TAG_ID, 0, 0);
        turret.update(null, vision);

        // 2. Получаем информацию о теге для графика
        AprilTagDetection tag = vision.getTarget(TARGET_TAG_ID);
        double errorDegrees = 0;

        if (tag != null) {
            // Bearing - это на сколько градусов тег смещен от центра камеры
            errorDegrees = tag.ftcPose.bearing;
        }

        // 3. ТЕЛЕМЕТРИЯ И ГРАФИКИ
        // В Dashboard нажмите на эти поля, чтобы увидеть график
        telemetry.addData("Vision Error (Bearing)", errorDegrees);
        telemetry.addData("Turret State", turret.getState());

        telemetry.addData("--- CURRENT VIS PID ---", "");
        telemetry.addData("P", SimpleTurret.VIS_P);
        telemetry.addData("D", SimpleTurret.VIS_D);

        if (tag != null) {
            telemetry.addData("STATUS", "🟢 LOCKED (Range: %.1f in)", tag.ftcPose.range);
        } else {
            telemetry.addData("STATUS", "🔴 SEARCHING (Show Tag!)");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}