package org.firstinspires.ftc.teamcode.Testiki;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "🧪 SHOOTER & VISION TUNER", group = "Test")
public class TestShooterTeleOp extends OpMode {

    // === ПЕРЕМЕННЫЕ ДЛЯ PANELS ===
    public static double TEST_RPM = 3000;
    public static double TEST_HOOD_POS = 0.3;

    // TRUE: Худ дергается при просадке RPM (тест PID)
    // FALSE: Худ стоит жестко (для сбора таблицы)
    public static boolean USE_COMPENSATION = false;

    // === СИСТЕМЫ ===
    private Vision vision;
    private Shooter shooter;
    private Hood hood;
    private Intake intake;
    private Claw claw;

    private static final int TARGET_TAG_ID = 24;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

        intake.stop();
        claw.close();

        telemetry.addLine("READY via Dashboard.");
        telemetry.addLine("GP1 A: Shoot (Open Claw) | GP1 B: Intake (Closed Claw)");
    }

    @Override
    public void init_loop() {
        vision.applyCombatSettings();

        AprilTagDetection tag = vision.getTarget(TARGET_TAG_ID);
        if (tag != null) {
            telemetry.addData("Tag 24 Detected", "YES");
            telemetry.addData("Dist", "%.2f inch", tag.ftcPose.range);
        } else {
            telemetry.addData("Tag 24 Detected", "NO");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Обновляем PIDF
        shooter.update();

        // 2. Ставим обороты и Худ из Dashboard
        shooter.setTargetRPM(TEST_RPM);
        hood.setBasePosition(TEST_HOOD_POS);

        // 3. Компенсация
        if (USE_COMPENSATION) {
            hood.update(shooter.getCurrentRPM(), TEST_RPM);
        } else {
            hood.update(TEST_RPM, TEST_RPM);
        }

        // === 4. ЛОГИКА ИНТЕЙКА И СТРЕЛЬБЫ ===
        boolean isShooting = gamepad1.a;
        boolean isIntaking = gamepad1.b;

        if (isShooting) {
            // СТРЕЛЬБА: Открываем клешню, толкаем
            claw.open();
            intake.intake();
        }
        else if (isIntaking) {
            // ЗАГРУЗКА: Закрываем клешню, засасываем
            claw.close();
            intake.intake();
        }
        else {
            // IDLE
            intake.stop();
            claw.close();
        }

        // 5. Данные Vision
        AprilTagDetection tag = vision.getTarget(TARGET_TAG_ID);
        double distance = (tag != null) ? tag.ftcPose.range : -1.0;

        // 6. Телеметрия
        double currentRPM = shooter.getCurrentRPM();
        double errorRPM = TEST_RPM - currentRPM;

        telemetry.addData("--- DATA COLLECTION ---", "");
        if (distance != -1) {
            telemetry.addData("📝 DISTANCE", "%.2f", distance);
            telemetry.addData("📝 ANGLE (Hood)", "%.3f", TEST_HOOD_POS);
        } else {
            telemetry.addData("DISTANCE", "NO TAG");
        }

        telemetry.addData("--- STATUS ---", "");
        telemetry.addData("Mode", isShooting ? "SHOOTING" : (isIntaking ? "INTAKING" : "IDLE"));
        telemetry.addData("Target RPM", "%.0f", TEST_RPM);
        telemetry.addData("Real RPM", "%.0f", currentRPM);
        telemetry.addData("Ready?", Math.abs(errorRPM) < Shooter.RPM_TOLERANCE ? "YES" : "NO");

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setTargetRPM(0);
        vision.stop();
        intake.stop();
    }
}