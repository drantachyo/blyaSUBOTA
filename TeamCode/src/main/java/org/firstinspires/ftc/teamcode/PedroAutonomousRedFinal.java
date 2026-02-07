package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Subsystems
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.math.ShooterMath;

// Pedro Pathing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Auto Red FINAL", group = "Competition")
public class PedroAutonomousRedFinal extends OpMode {

    // === НАСТРОЙКИ ===
    private static final int APRIL_TAG_ID = 24;
    private static final double OBELISK_X = 138.0;
    private static final double OBELISK_Y = 138.0;

    // Тайминги
    public static double SHOOT_TIME = 0.35;
    public static double INTAKE_WAIT_TIME = 0.7;

    // === SUBSYSTEMS ===
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

    // === ПЕРЕМЕННЫЕ ===
    private double lastKnownDistance = 70; // Дефолт
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;

    // === ФЛАГИ ОПТИМИЗАЦИИ ===
    private boolean actionTriggered = false;
    private boolean visionApplied = false;

    // === МАШИНА СОСТОЯНИЙ ===
    private enum AutoState {
        START,
        TO_SCORE_1, SCORE_1, GET_SAMPLE_1,
        TO_SCORE_2, SCORE_2, GET_SAMPLE_2, INTAKE_SAMPLE_2,
        TO_SCORE_3, SCORE_3, GET_SAMPLE_3, INTAKE_SAMPLE_3,
        TO_SCORE_4, SCORE_4, GET_SAMPLE_4, INTAKE_SAMPLE_4,
        TO_SCORE_5, SCORE_5,
        // Резервные стейты
        GET_CLOSE_BALLS, TO_SCORE_6, SCORE_6,
        PARK, IDLE
    }
    private AutoState currentState = AutoState.START;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        claw = new Claw(hardwareMap);
        hood = new Hood(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new SimpleTurret(hardwareMap);
        vision = new Vision(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 124, Math.toRadians(35)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initializing...");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (!visionApplied) {
            visionApplied = vision.applyCombatSettings(); // Применяем настройки (2ms выдержка)
            if (visionApplied) {
                panelsTelemetry.debug("Vision", "READY (Low Exposure Applied)");
            } else {
                panelsTelemetry.debug("Vision", "Waiting for camera stream...");
            }
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();
        claw.close();
        turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);
        setPathState(AutoState.TO_SCORE_1);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        Pose pose = follower.getPose();

        // --- БАЛЛИСТИКА И ПРЕД-РАСКРУТКА ---
        if (currentState.name().contains("SCORE")) {
            if (currentState.name().contains("TO_SCORE")) {
                double dist = vision.getDistanceFromTarget(APRIL_TAG_ID);
                if (dist != -1) lastKnownDistance = dist;
            }
            calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
            calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

            shooter.setTargetRPM(calculatedRPM);
            hood.setBasePosition(calculatedHood);
            hood.update(shooter.getCurrentRPM(), calculatedRPM);
            turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);
        } else {
            // Походный режим
            shooter.setTargetRPM(2000);
            hood.setBasePosition(Hood.LIMIT_BOTTOM);
            hood.update(shooter.getCurrentRPM(), 0);
            turret.update(pose, vision);
        }

        // --- МАШИНА СОСТОЯНИЙ ---
        switch (currentState) {
            // === ЦИКЛ 1 ===
            case TO_SCORE_1:
                if (!actionTriggered && timer.seconds() > 0.1) { intake.stop(); actionTriggered = true; }
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_1);
                }
                break;
            case SCORE_1:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_1);
                break;
            case GET_SAMPLE_1:
                // Клешня УЖЕ закрыта (в setPathState). Тут только включаем интейк.
                if(!actionTriggered && follower.getCurrentTValue() > 0.3) {
                    intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) setPathState(AutoState.TO_SCORE_2);
                break;

            // === ЦИКЛ 2 ===
            case TO_SCORE_2:
                if (!actionTriggered && timer.seconds() > 0.1) { intake.stop(); actionTriggered = true; }
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_2);
                }
                break;
            case SCORE_2:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_2);
                break;
            case GET_SAMPLE_2:
                if(!actionTriggered && follower.getCurrentTValue() > 0.3) {
                    intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.IntakeMiddleBalls2, true);
                    setPathStateInternal(AutoState.INTAKE_SAMPLE_2);
                }
                break;
            case INTAKE_SAMPLE_2:
                if(timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_3);
                break;

            // === ЦИКЛ 3 ===
            case TO_SCORE_3:
                if (!actionTriggered && timer.seconds() > 0.1) { intake.stop(); actionTriggered = true; }
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_3);
                }
                break;
            case SCORE_3:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_3);
                break;
            case GET_SAMPLE_3:
                if(!actionTriggered && follower.getCurrentTValue() > 0.3) {
                    intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.IntakeMiddleBalls3, true);
                    setPathStateInternal(AutoState.INTAKE_SAMPLE_3);
                }
                break;
            case INTAKE_SAMPLE_3:
                if(timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_4);
                break;

            // === ЦИКЛ 4 ===
            case TO_SCORE_4:
                if (!actionTriggered && timer.seconds() > 0.1) { intake.stop(); actionTriggered = true; }
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_4);
                }
                break;
            case SCORE_4:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_4);
                break;
            case GET_SAMPLE_4:
                if(!actionTriggered && follower.getCurrentTValue() > 0.3) {
                    intake.intake();
                    actionTriggered = true;
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.ToMiddleBalls4, true);
                    setPathStateInternal(AutoState.INTAKE_SAMPLE_4);
                }
                break;
            case INTAKE_SAMPLE_4:
                if(timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_5);
                break;

            // === ЦИКЛ 5 ===
            case TO_SCORE_5:
                if (!actionTriggered && timer.seconds() > 0.1) { intake.stop(); actionTriggered = true; }
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_5);
                }
                break;
            case SCORE_5:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.PARK);
                break;

            case PARK:
                if (!follower.isBusy()) currentState = AutoState.IDLE;
                break;
            case IDLE:
                break;
        }

        panelsTelemetry.debug("State", currentState);
        panelsTelemetry.update(telemetry);
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        timer.reset();
        actionTriggered = false;

        // --- ДЕЙСТВИЯ ПРИ ВХОДЕ ---

        // 1. СТРЕЛЬБА: Открыть клешню, включить интейк
        if (newState.name().contains("SCORE") && !newState.name().contains("TO")) {
            claw.open();
            intake.intake();
        }

        // 2. ПОЕЗДКА ЗА СЭМПЛОМ: Сразу закрыть клешню!
        if (newState.name().contains("GET_SAMPLE")) {
            claw.close();
        }

        switch (newState) {
            case TO_SCORE_1: follower.followPath(paths.ToScore1, true); break;
            case GET_SAMPLE_1: follower.followPath(paths.MiddleBalls1, true); break;

            case TO_SCORE_2: follower.followPath(paths.ToScore2, true); break;
            case GET_SAMPLE_2: follower.followPath(paths.MiddleBalls2, true); break;

            case TO_SCORE_3: follower.followPath(paths.ToScore3, true); break;
            case GET_SAMPLE_3: follower.followPath(paths.MiddleBalls3, true); break;

            case TO_SCORE_4: follower.followPath(paths.ToScore4, true); break;
            case GET_SAMPLE_4: follower.followPath(paths.ToMiddleBalls4, true); break;

            case TO_SCORE_5: follower.followPath(paths.ToScore5, true); break;

            // Резерв
            case GET_CLOSE_BALLS: follower.followPath(paths.ToCloseBalls, true); break;
            case TO_SCORE_6: follower.followPath(paths.ToScore6, true); break;

            case PARK:
                hood.setBasePosition(Hood.LIMIT_BOTTOM);
                shooter.setTargetRPM(0);
                turret.idle();
                intake.stop();
                follower.followPath(paths.BaseLeave, true);
                break;
        }
    }

    private void setPathStateInternal(AutoState state) {
        currentState = state;
        timer.reset();
        actionTriggered = false;
    }

    // === ВСЕ ПУТИ ===
    public static class Paths {
        public PathChain StartPosition, ToScore1, MiddleBalls1, ToScore2, MiddleBalls2, IntakeMiddleBalls2;
        public PathChain ToScore3, MiddleBalls3, IntakeMiddleBalls3, ToScore4, ToMiddleBalls4, IntakeMiddleBalls4;
        public PathChain ToScore5, ToCloseBalls, ToScore6, BaseLeave;

        public Paths(Follower follower) {
            StartPosition = follower.pathBuilder().addPath(new BezierLine(new Pose(123.000, 124.000), new Pose(123.000, 125.000))).setConstantHeadingInterpolation(Math.toRadians(35)).build();
            ToScore1 = follower.pathBuilder().addPath(new BezierLine(new Pose(123.000, 125.000), new Pose(84.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(-25)).build();
            MiddleBalls1 = follower.pathBuilder().addPath(new BezierCurve(new Pose(84.000, 76.000), new Pose(81.863, 67.979), new Pose(129.000, 57.000))).setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(0)).build();
            ToScore2 = follower.pathBuilder().addPath(new BezierCurve(new Pose(129.000, 57.000), new Pose(97.121, 70.963), new Pose(84.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25)).build();
            MiddleBalls2 = follower.pathBuilder().addPath(new BezierCurve(new Pose(84.000, 76.000), new Pose(110.077, 63.037), new Pose(132.000, 58.500))).setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(45)).build();
            IntakeMiddleBalls2 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 58.500), new Pose(132.000, 53.000))).setConstantHeadingInterpolation(Math.toRadians(45)).build();
            ToScore3 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 53.000), new Pose(84.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-25)).build();
            MiddleBalls3 = follower.pathBuilder().addPath(new BezierLine(new Pose(84.000, 76.000), new Pose(132.000, 58.500))).setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(45)).build();
            IntakeMiddleBalls3 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 58.500), new Pose(132.000, 53.000))).setConstantHeadingInterpolation(Math.toRadians(45)).build();
            ToScore4 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 53.000), new Pose(84.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-25)).build();
            ToMiddleBalls4 = follower.pathBuilder().addPath(new BezierLine(new Pose(84.000, 76.000), new Pose(132.000, 58.500))).setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(45)).build();
            IntakeMiddleBalls4 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 58.500), new Pose(132.000, 53.000))).setConstantHeadingInterpolation(Math.toRadians(45)).build();
            ToScore5 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 53.000), new Pose(90.000, 83.000))).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-25)).build();
            // 6-й цикл
            ToCloseBalls = follower.pathBuilder().addPath(new BezierLine(new Pose(90.000, 83.000), new Pose(128.000, 83.000))).setTangentHeadingInterpolation().build();
            ToScore6 = follower.pathBuilder().addPath(new BezierLine(new Pose(128.000, 83.000), new Pose(90.000, 83.000))).setTangentHeadingInterpolation().setReversed().build();
            BaseLeave = follower.pathBuilder().addPath(new BezierLine(new Pose(90.000, 83.000), new Pose(117.000, 72.000))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180)).build();
        }
    }
}