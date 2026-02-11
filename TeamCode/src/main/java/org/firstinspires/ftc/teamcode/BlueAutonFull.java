package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.math.PoseStorage;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "BLUE FULL (Turret Fixed -48) [FINAL]", group = "Competition")
public class BlueAutonFull extends OpMode {

    // =========================================================
    // НАСТРОЙКИ
    // =========================================================
    public static boolean SILENT_MODE = false;

    // Скорость
    public static double SPEED_TO_BALLS = 1.0;
    public static double SPEED_TO_SCORE = 0.65;

    // Тайминги
    public static double SHOOT_TIME = 1.6;
    public static double INTAKE_WAIT_TIME = 0.3;

    // Фиксированные значения
    public static double TARGET_RPM = 3850;
    public static double IDLE_RPM = 2500;
    public static double TARGET_HOOD = 0.0;

    // ВАЖНО: Угол турели относительно корпуса (по энкодеру)
    // Инвертирован для синей стороны (зеркально)
    public static double TARGET_TURRET_DEG = -48;

    // Системы
    private Follower follower;
    private Claw claw;
    private Hood hood;
    private Intake intake;
    private Shooter shooter;
    private SimpleTurret turret;
    private Vision vision;

    private Paths paths;
    private TelemetryManager panelsTelemetry;
    private ElapsedTime timer = new ElapsedTime();

    // Логика
    private boolean actionTriggered = false;

    // Старт (зеркально: X = 144 - 112.4 = 31.6, heading = 180 - 45 = 135)
    private Pose startPose = new Pose(31.6, 127.4, Math.toRadians(135));

    private enum AutoState {
        START,
        TO_SCORE_PRELOAD, SCORE_PRELOAD,
        TO_BALL_NEAR, WAIT_BALL_NEAR,
        TO_OPEN_GATE,
        TO_SCORE_NEAR, SCORE_NEAR,
        TO_BALL_MIDDLE, WAIT_BALL_MIDDLE, TO_SCORE_MIDDLE, SCORE_MIDDLE,
        TO_BALL_FAR, WAIT_BALL_FAR, TO_SCORE_FAR, SCORE_FAR,
        PARK, IDLE
    }

    private AutoState currentState = AutoState.START;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Инициализация железа
        claw = new Claw(hardwareMap);
        hood = new Hood(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new SimpleTurret(hardwareMap);
        vision = new Vision(hardwareMap);

        // Инициализация Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        // Сброс памяти и начальное состояние
        PoseStorage.currentPose = startPose;
        claw.close();

        // В ините башня расслаблена (или держит 0, если руками выставили)
        turret.idle();

        panelsTelemetry.debug("Status", "Waiting for Start...");
        panelsTelemetry.debug("Turret Mode", "Encoder Lock " + TARGET_TURRET_DEG);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();

        // Сразу ставим угол по энкодеру.
        // Метод setTargetAngle переводит турель в режим MANUAL,
        // где она игнорирует позицию робота на поле.
        turret.setTargetAngle(Math.toRadians(TARGET_TURRET_DEG));

        setPathState(AutoState.TO_SCORE_PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();

        // Получаем позу, но для турели она будет использоваться только формально
        Pose pose = follower.getPose();
        PoseStorage.currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());

        // --- ЛОГИКА ТУРЕЛИ ---
        // Если парковка или конец -> в ноль. Иначе -> -48 градусов.
        if (currentState == AutoState.PARK || currentState == AutoState.IDLE) {
            turret.setTargetAngle(0);
        } else {
            // Постоянно обновляем цель (на случай, если что-то сбилось),
            // но в MANUAL режиме это просто меняет переменную targetAngle.
            turret.setTargetAngle(Math.toRadians(TARGET_TURRET_DEG));
        }

        // ОБЯЗАТЕЛЬНО вызываем update.
        // Мы передаем pose и vision, потому что метод этого требует (по сигнатуре),
        // но так как мы использовали setTargetAngle, турель находится в состоянии
        // MANUAL
        // и внутри себя НЕ использует pose для расчета угла. Она работает чисто по
        // энкодеру.
        turret.update(pose, vision);

        // --- ОСТАЛЬНЫЕ СИСТЕМЫ ---
        hood.setBasePosition(TARGET_HOOD);

        if (currentState == AutoState.IDLE) {
            shooter.setTargetRPM(0);
            hood.setBasePosition(Hood.LIMIT_BOTTOM);
            intake.stop();
        } else {
            if (SILENT_MODE) {
                shooter.setTargetRPM(0);
                hood.update(0, 0);
                intake.stop();
            } else {
                if (currentState.name().contains("SCORE")) {
                    shooter.setTargetRPM(TARGET_RPM); // 3850
                    hood.update(shooter.getCurrentRPM(), TARGET_RPM);
                } else {
                    shooter.setTargetRPM(IDLE_RPM);
                    hood.update(shooter.getCurrentRPM(), 0);
                }

                if (currentState == AutoState.PARK)
                    intake.stop();
            }
        }

        // --- STATE MACHINE ---
        boolean isShooterReady = SILENT_MODE || shooter.isReady();

        switch (currentState) {
            case TO_SCORE_PRELOAD:
                if (!actionTriggered && timer.seconds() > 0.1) {
                    if (!SILENT_MODE)
                        intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady)
                    setPathState(AutoState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
                if (timer.seconds() > SHOOT_TIME)
                    setPathState(AutoState.TO_BALL_NEAR);
                break;

            case TO_BALL_NEAR:
                if (!actionTriggered && follower.getCurrentTValue() > 0.2) {
                    if (!SILENT_MODE)
                        intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy())
                    setPathStateInternal(AutoState.WAIT_BALL_NEAR);
                break;
            case WAIT_BALL_NEAR:
                if (timer.seconds() > INTAKE_WAIT_TIME)
                    setPathState(AutoState.TO_OPEN_GATE);
                break;

            case TO_OPEN_GATE:
                if (!actionTriggered && follower.getCurrentTValue() > 0.5) {
                    if (!SILENT_MODE)
                        intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy())
                    setPathState(AutoState.TO_SCORE_NEAR);
                break;

            case TO_SCORE_NEAR:
                if (!actionTriggered && timer.seconds() > 0.3) {
                    if (!SILENT_MODE)
                        intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady)
                    setPathState(AutoState.SCORE_NEAR);
                break;
            case SCORE_NEAR:
                if (timer.seconds() > SHOOT_TIME)
                    setPathState(AutoState.TO_BALL_MIDDLE);
                break;

            case TO_BALL_MIDDLE:
                if (!actionTriggered && follower.getCurrentTValue() > 0.2) {
                    if (!SILENT_MODE)
                        intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy())
                    setPathStateInternal(AutoState.WAIT_BALL_MIDDLE);
                break;
            case WAIT_BALL_MIDDLE:
                if (timer.seconds() > INTAKE_WAIT_TIME)
                    setPathState(AutoState.TO_SCORE_MIDDLE);
                break;
            case TO_SCORE_MIDDLE:
                if (!actionTriggered && timer.seconds() > 0.3) {
                    if (!SILENT_MODE)
                        intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady)
                    setPathState(AutoState.SCORE_MIDDLE);
                break;
            case SCORE_MIDDLE:
                if (timer.seconds() > SHOOT_TIME)
                    setPathState(AutoState.TO_BALL_FAR);
                break;

            case TO_BALL_FAR:
                if (!actionTriggered && follower.getCurrentTValue() > 0.2) {
                    if (!SILENT_MODE)
                        intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy())
                    setPathStateInternal(AutoState.WAIT_BALL_FAR);
                break;
            case WAIT_BALL_FAR:
                if (timer.seconds() > INTAKE_WAIT_TIME)
                    setPathState(AutoState.TO_SCORE_FAR);
                break;
            case TO_SCORE_FAR:
                if (!actionTriggered && timer.seconds() > 0.3) {
                    if (!SILENT_MODE)
                        intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady)
                    setPathState(AutoState.SCORE_FAR);
                break;
            case SCORE_FAR:
                if (timer.seconds() > SHOOT_TIME)
                    setPathState(AutoState.PARK);
                break;

            case PARK:
                if (!follower.isBusy())
                    currentState = AutoState.IDLE;
                break;
            case IDLE:
                break;
        }

        panelsTelemetry.debug("State", currentState);
        panelsTelemetry.debug("Turret Tgt", (currentState == AutoState.PARK) ? "0" : TARGET_TURRET_DEG);
        panelsTelemetry.update(telemetry);
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        timer.reset();
        actionTriggered = false;

        // Устанавливаем скорость в зависимости от того, куда едем
        if (newState.name().contains("TO_BALL") || newState.name().contains("PARK")) {
            follower.setMaxPower(SPEED_TO_BALLS);
        } else if (newState.name().contains("TO_SCORE")) {
            follower.setMaxPower(SPEED_TO_SCORE);
        }

        if (newState.name().contains("SCORE") && !newState.name().contains("TO")) {
            claw.open();
            if (!SILENT_MODE)
                intake.intake();
        } else if (newState.name().contains("TO_BALL")) {
            claw.close();
        }

        switch (newState) {
            case TO_SCORE_PRELOAD:
                follower.followPath(paths.ToScorePreload, true);
                break;

            case TO_BALL_NEAR:
                follower.followPath(paths.ToBallNear, true);
                break;
            case TO_OPEN_GATE:
                follower.followPath(paths.ToOpenGate, true);
                break;
            case TO_SCORE_NEAR:
                follower.followPath(paths.ToScoreNear, true);
                break;

            case TO_BALL_MIDDLE:
                follower.followPath(paths.ToBallMiddle, true);
                break;
            case TO_SCORE_MIDDLE:
                follower.followPath(paths.ToScoreMiddle, true);
                break;

            case TO_BALL_FAR:
                follower.followPath(paths.ToBallFar, true);
                break;
            case TO_SCORE_FAR:
                follower.followPath(paths.ToScoreFar, true);
                break;

            case PARK:
                follower.followPath(paths.LeaveBase, true);
                break;
        }
    }

    private void setPathStateInternal(AutoState state) {
        currentState = state;
        timer.reset();
        actionTriggered = false;
    }

    // === ПУТИ (ЗЕРКАЛЬНЫЕ — синяя сторона) ===
    // Формулы инверсии:
    // X_blue = 144 - X_red
    // Y_blue = Y_red (без изменений)
    // Heading_blue = 180° - Heading_red
    public static class Paths {
        // Зеркальные позиции: X = 144 - X_red
        public final Pose StartPose = new Pose(31.6, 127.4); // 144 - 112.4
        public final Pose ScorePose = new Pose(54, 87); // 144 - 90
        public final Pose NearBallPos = new Pose(30, 85); // 144 - 114
        public final Pose GatePos = new Pose(22, 76); // 144 - 122
        public final Pose MiddleBallPos = new Pose(25, 59); // 144 - 119
        public final Pose FarBallPos = new Pose(23, 36.5); // 144 - 121
        public final Pose ParkPose = new Pose(27, 85); // 144 - 117

        // Зеркальные углы: heading = 180° - heading_red
        double StartHeading = Math.toRadians(135); // 180 - 45
        double ScoreHeading = Math.toRadians(180); // 180 - 0
        double IntakeHeading = Math.toRadians(180); // 180 - 0

        public PathChain ToScorePreload;
        public PathChain ToBallNear, ToOpenGate, ToScoreNear;
        public PathChain ToBallMiddle, ToScoreMiddle;
        public PathChain ToBallFar, ToScoreFar;
        public PathChain LeaveBase;

        public Paths(Follower follower) {
            ToScorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(StartPose, ScorePose))
                    .setLinearHeadingInterpolation(StartHeading, ScoreHeading)
                    .build();

            ToBallNear = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, NearBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();

            ToOpenGate = follower.pathBuilder()
                    .addPath(new BezierLine(NearBallPos, GatePos))
                    .setLinearHeadingInterpolation(IntakeHeading, IntakeHeading)
                    .build();

            ToScoreNear = follower.pathBuilder()
                    .addPath(new BezierLine(GatePos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            ToBallMiddle = follower.pathBuilder()
                    // Контрольные точки Безье: X = 144 - X_red
                    // (76,79) -> (68,79), (97,57) -> (47,57)
                    .addPath(new BezierCurve(ScorePose, new Pose(68, 79), new Pose(47, 57), MiddleBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToScoreMiddle = follower.pathBuilder()
                    .addPath(new BezierLine(MiddleBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            ToBallFar = follower.pathBuilder()
                    // Контрольная точка: (87,36) -> (57,36)
                    .addPath(new BezierCurve(ScorePose, new Pose(57, 36), FarBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToScoreFar = follower.pathBuilder()
                    .addPath(new BezierLine(FarBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            LeaveBase = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, ParkPose))
                    // 180 - 180 = 0, зеркально красному (180 -> 0)
                    .setLinearHeadingInterpolation(ScoreHeading, Math.toRadians(0))
                    .build();
        }
    }
}
