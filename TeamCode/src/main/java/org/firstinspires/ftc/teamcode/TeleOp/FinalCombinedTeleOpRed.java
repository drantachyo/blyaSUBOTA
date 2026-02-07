package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.math.PoseStorage;
import org.firstinspires.ftc.teamcode.math.ShooterMath; // Импорт математики
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "RED TELEOP", group = "Final")
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
    private static final int TAG_ID = 24;
    private static final Pose START_POSE = new Pose(133.4, 8, 0);

    public enum RobotState { IDLE, INTAKE, OUTTAKE, PREP_SHOOT, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    // === ПЕРЕМЕННЫЕ ДЛЯ VARIABLE SHOOTING ===
    private double lastKnownDistance = 40.0; // Дефолт, если включились и не видим тег
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if (PoseStorage.currentPose != null) {
            // Если пришли из автонома - берем сохраненную позицию
            follower.setStartingPose(PoseStorage.currentPose);
            telemetry.addLine("Loaded Pose from Auto!");
        } else {
            // Если запустили ТелеОп "с нуля" - ставим дефолт
            follower.setStartingPose(START_POSE);
            telemetry.addLine("No Auto Pose found. Using Default.");
        }

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

        telemetry.addLine("READY with Variable Shooter Logic.");
    }

    @Override
    public void start() {
        if (PoseStorage.currentPose != null) {
            // Если пришли из автонома - берем сохраненную позицию
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            // Если запустили ТелеОп "с нуля" - ставим дефолт
            follower.setStartingPose(START_POSE);
        }
        follower.startTeleopDrive();
        vision.applyCombatSettings(); // Применяем настройки камеры на старте
    }

    @Override
    public void loop() {
        // 1. ОБНОВЛЕНИЕ СИСТЕМ
        follower.update();
        shooter.update();
        Pose pose = follower.getPose();

        // 2. ПОЛУЧЕНИЕ ДАННЫХ С КАМЕРЫ И РАСЧЕТ МАТЕМАТИКИ
        double currentDist = vision.getDistanceFromTarget(TAG_ID);

        if (currentDist != -1) {
            lastKnownDistance = currentDist; // Запоминаем новую дистанцию
        }
        // Если тег потерян, lastKnownDistance остается старым

        // Считаем параметры стрельбы
        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);


        // 3. УПРАВЛЕНИЕ ШАССИ (Pedro + Brake Logic)
        if (gamepad1.options) follower.setPose(START_POSE);

        if (gamepad1.b) {
            // ТОРМОЗ
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
        boolean aim = gamepad2.left_trigger > 0.1;
        boolean fire = gamepad2.right_trigger > 0.1;

        switch (currentState) {
            case IDLE:
                shooter.setTargetRPM(0);
                intake.stop();
                claw.close();
                // Худ можно опустить или оставить как есть, здесь оставляем в calculatedHood для готовности
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), 0); // Без нагрузки

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
                // Используем рассчитанные значения
                shooter.setTargetRPM(calculatedRPM);
                hood.setBasePosition(calculatedHood);

                // Передаем (текущий RPM, целевой RPM) для компенсации отдачи
                hood.update(shooter.getCurrentRPM(), calculatedRPM);

                if (!aim) currentState = RobotState.IDLE;
                else if (fire && shooter.isReady()) currentState = RobotState.SHOOTING;
                break;

            case SHOOTING:
                // Поддерживаем обороты и угол
                shooter.setTargetRPM(calculatedRPM);
                hood.setBasePosition(calculatedHood);
                hood.update(shooter.getCurrentRPM(), calculatedRPM);

                // Стреляем
                claw.open();
                intake.intake();

                if (!aim || !fire) currentState = aim ? RobotState.PREP_SHOOT : RobotState.IDLE;
                break;
        }

        // 5. ТУРЕЛЬ И ТЕЛЕМЕТРИЯ
        if (currentState == RobotState.PREP_SHOOT || currentState == RobotState.SHOOTING) {
            turret.track(TAG_ID, TARGET_X, TARGET_Y);
        } else {
            turret.idle();
        }
        turret.update(pose, vision);

        // Debug Info
        telemetry.addData("STATE", currentState);
        telemetry.addData("Vision", currentDist != -1 ? "LOCKED" : "SEARCHING");
        telemetry.addData("Dist Used", "%.1f inch", lastKnownDistance);
        telemetry.addData("Calc RPM", "%.0f", calculatedRPM);
        telemetry.addData("Calc Hood", "%.3f", calculatedHood);
        telemetry.update();
    }
}