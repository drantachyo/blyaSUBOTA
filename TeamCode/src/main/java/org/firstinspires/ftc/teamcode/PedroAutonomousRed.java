package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Subsystems
import org.firstinspires.ftc.teamcode.math.PoseStorage;
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

@Autonomous(name = "Pedro Auto Red", group = "Autonomous")
public class PedroAutonomousRed extends OpMode {

    // === КООРДИНАТЫ И НАСТРОЙКИ ===
    private static final double OBELISK_X = 138.0; // Для красного (зеркально) проверь X (обычно 6.0 или 138.0 в зависимости от системы)
    private static final double OBELISK_Y = 138.0;
    private static final int APRIL_TAG_ID = 24;

    // === ТАЙМИНГИ (Меняй их в Dashboard!) ===
    public static double SHOOT_TIME = 0.4;       // Время на выстрел
    public static double INTAKE_WAIT_TIME = 1.0; // Время ожидания на захвате сэмпла

    // === SUBSYSTEMS ===
    private Follower follower;
    private Claw claw;
    private Hood hood;
    private Intake intake;
    private Shooter shooter;
    private SimpleTurret turret;
    private Vision vision;

    private TelemetryManager panelsTelemetry;
    private Paths paths;
    private ElapsedTime timer = new ElapsedTime();

    // === ПЕРЕМЕННЫЕ ===
    private double lastKnownDistance = 60;
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;

    // === STATE MACHINE ===
    private enum AutoState {
        START,
        TO_SCORE_1, SCORE_1,
        GET_SAMPLE_1, INTAKE_SAMPLE_1,
        TO_SCORE_2, SCORE_2,
        GET_SAMPLE_2, INTAKE_SAMPLE_2,
        TO_SCORE_3, SCORE_3,
        GET_SAMPLE_3, INTAKE_SAMPLE_3,
        TO_SCORE_4, SCORE_4,
        GET_SAMPLE_4, INTAKE_SAMPLE_4,
        TO_SCORE_5, SCORE_5,
        PARK,
        IDLE
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
        // Проверь стартовую позицию для красного альянса
        follower.setStartingPose(new Pose(123, 124, Math.toRadians(35)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "READY.");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();
        vision.applyCombatSettings();
        claw.close();
        turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);
        setPathState(AutoState.TO_SCORE_1);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        Pose pose = follower.getPose();

        // --- ЛОГИКА ВИЖЕНА ---
        double currentDist = vision.getDistanceFromTarget(APRIL_TAG_ID);
        if (currentDist != -1) {
            lastKnownDistance = currentDist;
        }

        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

        // --- ЛОГИКА МЕХАНИЗМОВ ---
        if (currentState.name().contains("SCORE")) {
            shooter.setTargetRPM(calculatedRPM);
            hood.setBasePosition(calculatedHood);
            hood.update(shooter.getCurrentRPM(), calculatedRPM);
            turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);
        } else {
            shooter.setTargetRPM(2000);
            hood.setBasePosition(Hood.LIMIT_BOTTOM);
            hood.update(shooter.getCurrentRPM(), 0);
            turret.update(pose, vision);
        }

        // --- МАШИНА СОСТОЯНИЙ ---
        switch (currentState) {
            case TO_SCORE_1:
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_1);
                }
                break;

            case SCORE_1:
                claw.open();
                intake.intake();
                // ИСПОЛЬЗУЕМ ПЕРЕМЕННУЮ
                if (timer.seconds() > SHOOT_TIME) {
                    setPathState(AutoState.GET_SAMPLE_1);
                }
                break;

            case GET_SAMPLE_1: // MiddleBalls1
                if(follower.getCurrentTValue() > 0.3) {
                    intake.intake();
                    claw.close();
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.92) {
                    setPathState(AutoState.TO_SCORE_2);
                }
                break;

            case TO_SCORE_2:
                if (timer.seconds() > 0.1) intake.stop();
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_2);
                }
                break;

            case SCORE_2:
                claw.open();
                intake.intake();
                // ИСПОЛЬЗУЕМ ПЕРЕМЕННУЮ
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_2);
                break;

            case GET_SAMPLE_2: // MiddleBalls2
                if(follower.getCurrentTValue() > 0.3){
                    intake.intake();
                    claw.close();
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.IntakeMiddleBalls2, true);
                    setPathState(AutoState.INTAKE_SAMPLE_2);
                }
                break;

            case INTAKE_SAMPLE_2:
                // ЖДЕМ ВРЕМЯ ЗАХВАТА
                if(timer.seconds() > INTAKE_WAIT_TIME) {
                    setPathState(AutoState.TO_SCORE_3);
                }
                break;

            case TO_SCORE_3:
                claw.close();
                if(timer.seconds() > 0.5) intake.stop();
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_3);
                }
                break;

            case SCORE_3:
                claw.open();
                intake.intake();
                // ИСПОЛЬЗУЕМ ПЕРЕМЕННУЮ
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_3);
                break;

            case GET_SAMPLE_3: // MiddleBalls3
                if(follower.getCurrentTValue() > 0.3){
                    intake.intake();
                    claw.close();
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.IntakeMiddleBalls3, true);
                    setPathState(AutoState.INTAKE_SAMPLE_3);
                }
                break;

            case INTAKE_SAMPLE_3:
                if(timer.seconds() > INTAKE_WAIT_TIME) {
                    setPathState(AutoState.TO_SCORE_4);
                }
                break;

            case TO_SCORE_4:
                claw.close();
                if(timer.seconds() > 0.5) intake.stop();
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_4);
                }
                break;

            case SCORE_4:
                claw.open();
                intake.intake();
                // ИСПОЛЬЗУЕМ ПЕРЕМЕННУЮ
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.GET_SAMPLE_4);
                break;

            case GET_SAMPLE_4: // MiddleBalls4
                if(follower.getCurrentTValue() > 0.3){
                    intake.intake();
                    claw.close();
                }
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) {
                    follower.followPath(paths.IntakeMiddleBalls4, true);
                    setPathState(AutoState.INTAKE_SAMPLE_4);
                }
                break;

            case INTAKE_SAMPLE_4:
                if(timer.seconds() > INTAKE_WAIT_TIME) {
                    setPathState(AutoState.TO_SCORE_5);
                }
                break;

            case TO_SCORE_5:
                claw.close();
                if(timer.seconds() > 0.4) intake.stop();
                if ((!follower.isBusy() || follower.getCurrentTValue() > 0.95) && shooter.isReady()) {
                    setPathState(AutoState.SCORE_5);
                }
                break;

            case SCORE_5:
                claw.open();
                intake.intake();
                // ИСПОЛЬЗУЕМ ПЕРЕМЕННУЮ
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.PARK);
                break;

            case PARK:
                hood.setBasePosition(Hood.LIMIT_BOTTOM);
                shooter.setTargetRPM(0);
                turret.idle();
                intake.stop();
                if (!follower.isBusy()) currentState = AutoState.IDLE;
                break;

            case IDLE:
                break;
        }

        // Telemetry
        panelsTelemetry.debug("State", currentState);
        panelsTelemetry.debug("Timer", String.format("%.1f", timer.seconds()));
        panelsTelemetry.debug("T-Value", String.format("%.2f", follower.getCurrentTValue()));
        panelsTelemetry.debug("RPM", String.format("%.0f", shooter.getCurrentRPM()));
        panelsTelemetry.update(telemetry);
        PoseStorage.currentPose = follower.getPose();
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        timer.reset();

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
            case PARK: follower.followPath(paths.BaseLeave, true); break;
        }
    }

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
            ToCloseBalls = follower.pathBuilder().addPath(new BezierLine(new Pose(90.000, 83.000), new Pose(128.000, 83.000))).setTangentHeadingInterpolation().build();
            ToScore6 = follower.pathBuilder().addPath(new BezierLine(new Pose(128.000, 83.000), new Pose(90.000, 83.000))).setTangentHeadingInterpolation().setReversed().build();
            BaseLeave = follower.pathBuilder().addPath(new BezierLine(new Pose(90.000, 83.000), new Pose(117.000, 72.000))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180)).build();
        }
    }
}