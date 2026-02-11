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
import org.firstinspires.ftc.teamcode.math.ShooterMath;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "RED FULL (Trajectories 1 + Logic 2)", group = "Competition")
public class RedAutonCombined extends OpMode {

    // =========================================================
    // НАСТРОЙКИ (Смешанные)
    // =========================================================
    public static boolean SILENT_MODE = false;
    public static double SPEED = 0.6;

    // Тайминги
    public static double SHOOT_TIME = 1.8;
    public static double INTAKE_WAIT_TIME = 0.3;
    public static double IDLE_RPM = 2500;

    // Vision Settings (из второго кода)
    private static final int APRIL_TAG_ID = 24;
    private static final double OBELISK_X = 138.0;
    private static final double OBELISK_Y = 138.0;

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

    // Переменные логики (из второго кода)
    private double lastKnownDistance = 70; // Дефолтная дистанция старта
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;
    private boolean actionTriggered = false;

    // Старт (из первого кода)
    private Pose startPose = new Pose(112.4, 127.4, Math.toRadians(45));

    // Состояния (структура первого кода, так как траектории оттуда)
    private enum AutoState {
        START,
        TO_SCORE_PRELOAD, SCORE_PRELOAD,
        TO_BALL_NEAR, WAIT_BALL_NEAR, TO_SCORE_NEAR, SCORE_NEAR,
        TO_OPEN_GATE, // Уникально для первого кода
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

        // Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(SPEED);

        paths = new Paths(follower);

        PoseStorage.currentPose = startPose;
        claw.close();
        turret.idle();

        panelsTelemetry.debug("Status", "Ready. Logic: TRACKING. Paths: FULL.");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Можно добавить предварительный прогрев Vision здесь
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();
        vision.applyCombatSettings(); // Включаем настройки камеры для боя
        setPathState(AutoState.TO_SCORE_PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        Pose pose = follower.getPose();
        PoseStorage.currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());

        // 1. ЛОГИКА VISION И МАТЕМАТИКИ (из второго кода)
        // Мы постоянно считаем дистанцию, даже во время движения
        double dist = vision.getDistanceFromTarget(APRIL_TAG_ID);
        if (dist != -1) {
            lastKnownDistance = dist;
        }

        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

        // 2. УПРАВЛЕНИЕ СИСТЕМАМИ
        if (currentState == AutoState.IDLE || currentState == AutoState.PARK) {
            shooter.setTargetRPM(0);
            hood.setBasePosition(Hood.LIMIT_BOTTOM);
            turret.setTargetAngle(0); // В парковке турель прямо
            intake.stop();
        } else {
            // АКТИВНЫЙ ТРЕКИНГ (из второго кода)
            // Вместо фиксированных 44.5 градусов мы используем aim на координаты
            turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);

            // Худ всегда готовится
            hood.setBasePosition(calculatedHood);

            if (SILENT_MODE) {
                shooter.setTargetRPM(0);
                hood.update(0, 0);
            } else {
                if (currentState.name().contains("SCORE")) {
                    // СТРЕЛЬБА: используем рассчитанные RPM
                    shooter.setTargetRPM(calculatedRPM);
                    hood.update(shooter.getCurrentRPM(), calculatedRPM);
                } else {
                    // ЕЗДА: держим IDLE обороты
                    shooter.setTargetRPM(IDLE_RPM);
                    hood.update(shooter.getCurrentRPM(), 0);
                }
            }
        }

        // 3. STATE MACHINE
        // Переходы по состояниям из ПЕРВОГО кода, но условия (timer/isBusy) из ВТОРОГО
        boolean isShooterReady = SILENT_MODE || shooter.isReady();

        switch (currentState) {
            case TO_SCORE_PRELOAD:
                if (!actionTriggered && timer.seconds() > 0.1) {
                    if (!SILENT_MODE) intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady) setPathState(AutoState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_BALL_NEAR);
                break;

            case TO_BALL_NEAR:
                if(!actionTriggered && follower.getCurrentTValue() > 0.2) {
                    if (!SILENT_MODE) intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_NEAR);
                break;
            case WAIT_BALL_NEAR:
                if (timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_NEAR);
                break;
            case TO_SCORE_NEAR:
                if (!actionTriggered && timer.seconds() > 0.3) {
                    if (!SILENT_MODE) intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady) setPathState(AutoState.SCORE_NEAR);
                break;
            case SCORE_NEAR:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_OPEN_GATE);
                break;

            // --- УНИКАЛЬНАЯ ЧАСТЬ ПЕРВОГО КОДА (GATE) ---
            case TO_OPEN_GATE:
                // Едем открывать ворота, включаем интейк заранее
                if(!actionTriggered && follower.getCurrentTValue() > 0.5) {
                    if (!SILENT_MODE) intake.intake();
                    actionTriggered = true;
                }
                // Здесь нет стрельбы, сразу едем к Middle Ball
                if (!follower.isBusy()) setPathStateInternal(AutoState.TO_BALL_MIDDLE);
                break;

            case TO_BALL_MIDDLE:
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_MIDDLE);
                break;
            case WAIT_BALL_MIDDLE:
                if (timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_MIDDLE);
                break;
            case TO_SCORE_MIDDLE:
                if (!actionTriggered && timer.seconds() > 0.3) {
                    if (!SILENT_MODE) intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady) setPathState(AutoState.SCORE_MIDDLE);
                break;
            case SCORE_MIDDLE:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_BALL_FAR);
                break;

            case TO_BALL_FAR:
                if(!actionTriggered && follower.getCurrentTValue() > 0.2) {
                    if (!SILENT_MODE) intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_FAR);
                break;
            case WAIT_BALL_FAR:
                if (timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_FAR);
                break;
            case TO_SCORE_FAR:
                if (!actionTriggered && timer.seconds() > 0.3) {
                    if (!SILENT_MODE) intake.stop();
                    actionTriggered = true;
                }
                if (!follower.isBusy() && isShooterReady) setPathState(AutoState.SCORE_FAR);
                break;
            case SCORE_FAR:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.PARK);
                break;

            case PARK:
                if (!follower.isBusy()) currentState = AutoState.IDLE;
                break;
            case IDLE:
                break;
        }

        // Обновляем турель с учетом координат робота (важно для активного трекинга)
        turret.update(pose, vision);

        panelsTelemetry.debug("State", currentState);
        panelsTelemetry.debug("Dist", "%.1f", lastKnownDistance);
        panelsTelemetry.debug("RPM", "%.0f", calculatedRPM);
        panelsTelemetry.update(telemetry);
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        timer.reset();
        actionTriggered = false;
        follower.setMaxPower(SPEED);

        if (newState.name().contains("SCORE") && !newState.name().contains("TO")) {
            claw.open();
            if (!SILENT_MODE) intake.intake(); // Доталкиваем
        } else if (newState.name().contains("TO_BALL") || newState.name().contains("GATE")) {
            claw.close();
        }

        // ПУТИ ИЗ ПЕРВОГО КОДА
        switch (newState) {
            case TO_SCORE_PRELOAD: follower.followPath(paths.ToScorePreload, true); break;

            case TO_BALL_NEAR:    follower.followPath(paths.ToBallNear, true); break;
            case TO_SCORE_NEAR:   follower.followPath(paths.ToScoreNear, true); break;

            case TO_OPEN_GATE:    follower.followPath(paths.ToOpenGate, true); break;
            case TO_BALL_MIDDLE:  follower.followPath(paths.ToBallMiddle, true); break;
            case TO_SCORE_MIDDLE: follower.followPath(paths.ToScoreMiddle, true); break;

            case TO_BALL_FAR:     follower.followPath(paths.ToBallFar, true); break;
            case TO_SCORE_FAR:    follower.followPath(paths.ToScoreFar, true); break;

            case PARK:            follower.followPath(paths.LeaveBase, true); break;
        }
    }

    private void setPathStateInternal(AutoState state) {
        currentState = state;
        timer.reset();
        actionTriggered = false;

        if (state == AutoState.TO_BALL_MIDDLE) {
            // Если мы перешли сюда из Gate, запускаем путь
            follower.followPath(paths.ToBallMiddle, true);
        }
    }

    // =========================================================
    // ПУТИ (Полная копия из ПЕРВОГО кода)
    // =========================================================
    public static class Paths {
        public final Pose StartPose   = new Pose(112.4, 127.4);
        public final Pose ScorePose   = new Pose(90,87);
        public final Pose NearBallPos = new Pose(114, 85);
        public final Pose GatePos       = new Pose(110, 78);
        public final Pose MiddleBallPos = new Pose(119, 59);
        public final Pose FarBallPos    = new Pose(119, 36.5);
        public final Pose ParkPose    = new Pose(117, 85);

        double StartHeading = Math.toRadians(45);
        double ScoreHeading = Math.toRadians(0);
        double IntakeHeading = Math.toRadians(0);

        public PathChain ToScorePreload;
        public PathChain ToBallNear, ToScoreNear;
        public PathChain ToOpenGate, ToBallMiddle, ToScoreMiddle;
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
            ToScoreNear = follower.pathBuilder()
                    .addPath(new BezierLine(NearBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            ToOpenGate = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, GatePos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToBallMiddle = follower.pathBuilder()
                    .addPath(new BezierCurve(GatePos, new Pose(76, 79), new Pose(97, 57), MiddleBallPos))
                    .setLinearHeadingInterpolation(IntakeHeading, IntakeHeading)
                    .build();
            ToScoreMiddle = follower.pathBuilder()
                    .addPath(new BezierLine(MiddleBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            ToBallFar = follower.pathBuilder()
                    .addPath(new BezierCurve(ScorePose, new Pose(87, 36), FarBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToScoreFar = follower.pathBuilder()
                    .addPath(new BezierLine(FarBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            LeaveBase = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, ParkPose))
                    .setLinearHeadingInterpolation(ScoreHeading, Math.toRadians(180))
                    .build();
        }
    }
}