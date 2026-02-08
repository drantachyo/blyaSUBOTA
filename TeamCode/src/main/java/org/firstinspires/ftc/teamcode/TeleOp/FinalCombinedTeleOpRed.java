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

@TeleOp(name = "RED TELEOP FINAL", group = "Final")
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

    // === КООРДИНАТЫ И НАСТРОЙКИ ===
    private static final double TARGET_X = 138;
    private static final double TARGET_Y = 138;
    private static final int TAG_ID = 24; // ID апрельтага на красной стороне
    private static final Pose START_POSE = new Pose(133.4, 8, 0);

    public enum RobotState { IDLE, INTAKE, OUTTAKE, PREP_SHOOT, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    // === ПЕРЕМЕННЫЕ ДЛЯ VARIABLE SHOOTING ===
    private double lastKnownDistance = 40.0; // Дефолт, если включились и не видим тег
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;

    @Override
    public void init() {
        // Инициализация Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
            telemetry.addLine("Loaded Pose from Auto!");
        } else {
            follower.setStartingPose(START_POSE);
            telemetry.addLine("No Auto Pose found. Using Default.");
        }

        // Инициализация моторов шасси для ручного тормоза
        motors = new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "lf"),
                hardwareMap.get(DcMotorEx.class, "lr"),
                hardwareMap.get(DcMotorEx.class, "rf"),
                hardwareMap.get(DcMotorEx.class, "rr")
        };

        // Инициализация подсистем
        turret = new SimpleTurret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
        hood = new Hood(hardwareMap);
        claw = new Claw(hardwareMap);

        claw.close(); // Старт с закрытой клешней

        telemetry.addLine("READY: Hold Logic Implementation");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        vision.applyCombatSettings(); // Оптимизация камеры
    }

    @Override
    public void loop() {
        // 1. ОБНОВЛЕНИЕ БАЗОВЫХ СИСТЕМ
        follower.update();
        shooter.update();
        Pose pose = follower.getPose();

        // 2. РАСЧЕТ ДИСТАНЦИИ И ПАРАМЕТРОВ СТРЕЛЬБЫ
        // Если мы не стреляем прямо сейчас, можно обновлять дистанцию
        if (currentState != RobotState.SHOOTING) {
            double currentDist = vision.getDistanceFromTarget(TAG_ID);
            if (currentDist != -1) {
                lastKnownDistance = currentDist;
            }
        }

        // Пересчитываем RPM и Hood всегда на основе последней известной дистанции
        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

        // 3. УПРАВЛЕНИЕ ШАССИ (Pedro + Brake Logic)
        if (gamepad1.options) {
            follower.setPose(START_POSE);
            PoseStorage.currentPose = START_POSE;
        }

        if (gamepad1.b) {
            // ТОРМОЗ (Lock)
            follower.setTeleOpDrive(0, 0, 0, false);
            if (!isBraking) {
                for (DcMotorEx m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                isBraking = true;
            }
        } else {
            // ЕЗДА
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

        // 4. ЛОГИКА МЕХАНИЗМОВ (STATE MACHINE)
        boolean aim = gamepad2.left_trigger > 0.1;   // Левый курок - Preparing
        boolean fire = gamepad2.right_trigger > 0.1; // Правый курок - Shooting

        switch (currentState) {
            case IDLE:
                shooter.setTargetRPM(0);
                intake.stop();
                claw.close();
                // Держим худ наготове, но не нагружаем сервы лишний раз
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
                // Раскручиваемся
                shooter.setTargetRPM(calculatedRPM);
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), calculatedRPM);

                if (!aim) currentState = RobotState.IDLE;
                else if (fire && shooter.isReady()) currentState = RobotState.SHOOTING;
                break;

            case SHOOTING:
                // Поддерживаем обороты
                shooter.setTargetRPM(calculatedRPM);
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), calculatedRPM);

                // Открываем огонь
                claw.open();
                intake.intake(); // Подача кольца

                // Если отпустили курок стрельбы - обратно в Prep, если оба - в Idle
                if (!aim || !fire) currentState = aim ? RobotState.PREP_SHOOT : RobotState.IDLE;
                break;
        }

        // 5. ЛОГИКА ТУРЕЛИ (ОПТИМИЗИРОВАННАЯ)
        if (currentState == RobotState.SHOOTING) {
            // ВАЖНО: Когда стреляем, отключаем Vision и Одометрию, просто держим угол (PID).
            // Это экономит ресурсы процессора для Shooter PID.
            turret.hold();
        }
        else if (currentState == RobotState.PREP_SHOOT) {
            // Когда целимся (но не стреляем), активно используем камеру и одометрию.
            turret.track(TAG_ID, TARGET_X, TARGET_Y);
        }
        else {
            // В остальных случаях - парковка в ноль (или выключение).
            turret.idle();
        }

        // Вызываем update один раз в конце
        // Если turret.hold() был вызван, update пропустит тяжелые вычисления.
        turret.update(pose, vision);

        // 6. ТЕЛЕМЕТРИЯ
        telemetry.addData("STATE", currentState);
        telemetry.addData("Turret", turret.getState()); // Покажет MANUAL во время стрельбы
        telemetry.addData("Dist", "%.1f (Target: %d)", lastKnownDistance, TAG_ID);
        telemetry.addData("Shooter", "RPM: %.0f / Tgt: %.0f", shooter.getCurrentRPM(), calculatedRPM);
        telemetry.update();
    }
}