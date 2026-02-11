package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.math.PoseStorage;
import org.firstinspires.ftc.teamcode.math.ShooterMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "RED TELEOP ULTIMATE", group = "Final")
public class FinalCombinedTeleOpRed extends OpMode {

    // === СИСТЕМЫ ===
    private Follower follower;
    private SimpleTurret turret;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;
    private Hood hood;
    private Claw claw;

    private DcMotorEx[] motors;
    private boolean isBraking = false;

    // === КООРДИНАТЫ И НАСТРОЙКИ (КРАСНЫЕ) ===
    private static final double TARGET_X = 138;
    private static final double TARGET_Y = 138;
    private static final int TAG_ID = 24;

    private static final Pose START_POSE = new Pose(9.6, 8, Math.toRadians(180));
    private Pose savedAutoPose = null;

    public enum RobotState { IDLE, INTAKE, OUTTAKE, PREP_SHOOT, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    // === ПЕРЕМЕННЫЕ ДЛЯ СТРЕЛЬБЫ ===
    private double lastKnownDistance = 40.0;
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;

    // === FAILSAFE (РУЧНОЙ РЕЖИМ) ===
    private boolean manualMode = false; // Флаг ручного режима
    private boolean lastX = false;      // Для переключателя кнопки

    // Флаг для камеры
    private boolean cameraSettingsApplied = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        if (PoseStorage.currentPose != null) {
            savedAutoPose = new Pose(
                    PoseStorage.currentPose.getX(),
                    PoseStorage.currentPose.getY(),
                    PoseStorage.currentPose.getHeading()
            );
            follower.setStartingPose(savedAutoPose);
            telemetry.addLine("✅ LOADED AUTO POSE");
        } else {
            savedAutoPose = null;
            follower.setStartingPose(START_POSE);
            telemetry.addLine("⚠️ NO AUTO POSE. USING DEFAULT (RED).");
        }

        PoseStorage.currentPose = null;

        motors = new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "lf"),
                hardwareMap.get(DcMotorEx.class, "lr"),
                hardwareMap.get(DcMotorEx.class, "rf"),
                hardwareMap.get(DcMotorEx.class, "rr")
        };

        turret = new SimpleTurret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
        hood = new Hood(hardwareMap);
        claw = new Claw(hardwareMap);

        claw.close();
        telemetry.addData("Side", "RED");
        telemetry.addLine("⏳ Waiting for camera...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Pose p = follower.getPose();
        telemetry.addData("Waiting Start", "X:%.1f Y:%.1f H:%.1f",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));

        if (!cameraSettingsApplied) {
            boolean success = vision.applyCombatSettings();
            if (success) {
                cameraSettingsApplied = true;
                telemetry.addLine("✅ Camera settings applied!");
            } else {
                telemetry.addLine("⏳ Waiting for camera to stream...");
            }
        } else {
            telemetry.addLine("✅ Camera ready!");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        if (savedAutoPose != null) {
            follower.setPose(savedAutoPose);
        }
    }

    @Override
    public void loop() {
        // 1. ОБНОВЛЕНИЕ СИСТЕМ
        follower.update();
        shooter.update();
        Pose currentPose = follower.getPose();

        // 2. ЛОГИКА ПЕРЕКЛЮЧАТЕЛЯ (FAILSAFE TOGGLE)
        boolean currentX = gamepad2.x; // Используем Gamepad 2
        if (currentX && !lastX) {
            manualMode = !manualMode; // Переключаем режим
        }
        lastX = currentX;

        // 3. РАСЧЕТ ДИСТАНЦИИ И БАЛЛИСТИКИ
        if (manualMode) {
            // === РУЧНОЙ РЕЖИМ ===
            // Статичные значения, если камера сломалась или потерялась
            calculatedRPM = 3800;
            calculatedHood = 0.2;
        } else {
            // === АВТО РЕЖИМ ===
            double dist = vision.getDistanceFromTarget(TAG_ID);
            if (dist != -1) {
                lastKnownDistance = dist;
            }
            calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
            calculatedHood = ShooterMath.calculateHood(lastKnownDistance);
        }

        // 4. СБРОС КООРДИНАТ
        if (gamepad1.options) {
            follower.setPose(START_POSE);
        }

        // 5. УПРАВЛЕНИЕ ШАССИ + ТОРМОЗ
        // ИЗМЕНЕНИЕ: Убран авто-тормоз при стрельбе
        boolean manualBrake = gamepad1.b;

        if (manualBrake) {
            follower.setTeleOpDrive(0, 0, 0, false);
            if (!isBraking) {
                for (DcMotorEx m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                isBraking = true;
            }
        } else {
            if (isBraking) {
                for (DcMotorEx m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                isBraking = false;
            }
            double speedMultiplier = gamepad1.right_bumper ? 0.3 : 1.0;
            double turn = Math.pow((gamepad1.left_trigger - gamepad1.right_trigger), 3);

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speedMultiplier,
                    -gamepad1.left_stick_x * speedMultiplier,
                    turn * 0.5,
                    false
            );
        }

        // 6. STATE MACHINE
        boolean aim = gamepad2.left_trigger > 0.1;
        boolean fire = gamepad2.right_trigger > 0.1;

        switch (currentState) {
            case IDLE:
                shooter.setTargetRPM(0);
                intake.stop();
                claw.close();
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), 0);

                if (aim) currentState = RobotState.PREP_SHOOT;
                else if (gamepad2.b) currentState = RobotState.INTAKE;
                else if (gamepad2.dpad_down) currentState = RobotState.OUTTAKE;
                break;

            case INTAKE:
                intake.intake();
                claw.close();
                if (!gamepad2.b) currentState = RobotState.IDLE;
                break;

            case OUTTAKE:
                intake.outtake();
                claw.close();
                if (!gamepad2.dpad_down) currentState = RobotState.IDLE;
                break;

            case PREP_SHOOT:
                shooter.setTargetRPM(calculatedRPM);
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), calculatedRPM);

                if (!aim) currentState = RobotState.IDLE;
                else if (fire && shooter.isReady()) currentState = RobotState.SHOOTING;
                break;

            case SHOOTING:
                shooter.setTargetRPM(calculatedRPM);
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), calculatedRPM);

                claw.open();
                intake.intake();

                if (!aim || !fire) currentState = aim ? RobotState.PREP_SHOOT : RobotState.IDLE;
                break;
        }

        // 7. УПРАВЛЕНИЕ ТУРЕЛЬЮ
        if (manualMode) {
            // === FAILSAFE TURRET ===
            // Ставим турель в 45 градусов
            turret.setTargetAngle(-45);
        }
        else if (currentState == RobotState.PREP_SHOOT || currentState == RobotState.SHOOTING) {
            // Обычный трекинг
            turret.track(TAG_ID, TARGET_X, TARGET_Y);
        } else {
            turret.idle();
        }

        turret.update(currentPose, vision);

        // 8. ТЕЛЕМЕТРИЯ
        telemetry.addData("═══════════════════════", "");
        if (manualMode) {
            telemetry.addData("⚠️ MODE", "MANUAL FAILSAFE");
            telemetry.addData("Target", "Static 45°, 3800 RPM");
        } else {
            telemetry.addData("✅ MODE", "AUTO TRACKING");
        }
        telemetry.addData("STATE", currentState);
        telemetry.addData("═══════════════════════", "");

        if (!manualMode) {
            double dist = vision.getDistanceFromTarget(TAG_ID);
            if (dist != -1) telemetry.addData("📷 CAMERA", "✅ TAG VISIBLE (%.1f)", dist);
            else telemetry.addData("📷 CAMERA", "❌ TAG LOST (Last: %.1f)", lastKnownDistance);
        }

        telemetry.addData("Calc RPM", "%.0f", calculatedRPM);
        telemetry.addData("Calc Hood", "%.3f", calculatedHood);
        telemetry.update();
    }
}