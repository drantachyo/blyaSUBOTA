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
import org.firstinspires.ftc.teamcode.math.ShooterMath;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "RED 18 STOP & SHOOT", group = "Competition")
public class RedAutonStopShoot extends OpMode {

    // === ТАЙМИНГИ ===
    public static double SHOOT_TIME = 2;
    public static double INTAKE_WAIT_TIME = 0.6; // Обычное ожидание
    public static double INTAKE_WAIT_GATE = 1.2; // Долгое ожидание на воротах (Gate)
    public static double IDLE_RPM = 2500;

    // === VISION ===
    private static final int APRIL_TAG_ID = 24;
    private static final double OBELISK_X = 138.0;
    private static final double OBELISK_Y = 138.0;

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

    private double lastKnownDistance = 70;
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;
    private boolean visionApplied = false;
    private boolean actionTriggered = false;

    // === STATE MACHINE ===
    private enum AutoState {
        START,
        TO_SCORE_PRELOAD, SCORE_PRELOAD,
        TO_BALL_NEAR, WAIT_BALL_NEAR, TO_SCORE_NEAR, SCORE_NEAR,
        TO_BALL_MIDDLE, WAIT_BALL_MIDDLE, TO_SCORE_MIDDLE, SCORE_MIDDLE,
        TO_BALL_GATE, WAIT_BALL_GATE, TO_SCORE_GATE, SCORE_GATE,
        TO_BALL_FAR, WAIT_BALL_FAR, TO_SCORE_FAR, SCORE_FAR,
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
        paths = new Paths(follower);

        // Старт с углом 45 градусов
        follower.setStartingPose(new Pose(112.4, 127.4, Math.toRadians(45)));

        panelsTelemetry.debug("Status", "Waiting for Start...");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (!visionApplied) {
            visionApplied = vision.applyCombatSettings();
            if(visionApplied) panelsTelemetry.debug("Vision", "READY");
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();
        claw.close();
        turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);
        setPathState(AutoState.TO_SCORE_PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        Pose pose = follower.getPose();

        // 1. Vision
        double dist = vision.getDistanceFromTarget(APRIL_TAG_ID);
        if (dist != -1) lastKnownDistance = dist;
        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

        // 2. Subsystems
        if (currentState == AutoState.IDLE) {
            shooter.setTargetRPM(0);
            turret.setTargetAngle(0);
            hood.setBasePosition(Hood.LIMIT_BOTTOM);
            intake.stop();
        } else {
            turret.track(APRIL_TAG_ID, OBELISK_X, OBELISK_Y);
            hood.setBasePosition(calculatedHood);

            if (currentState.name().contains("SCORE")) {
                shooter.setTargetRPM(calculatedRPM);
                hood.update(shooter.getCurrentRPM(), calculatedRPM);
            } else {
                shooter.setTargetRPM(IDLE_RPM);
                hood.update(shooter.getCurrentRPM(), 0);
            }
            if (currentState == AutoState.PARK) intake.stop();
        }

        // 3. State Machine
        switch (currentState) {
            // PRELOAD
            case TO_SCORE_PRELOAD:
                if (!actionTriggered && timer.seconds() > 0.1) { intake.stop(); actionTriggered = true; }
                // ЖДЕМ ПОЛНОЙ ОСТАНОВКИ (!isBusy)
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_BALL_NEAR);
                break;

            // 1. NEAR
            case TO_BALL_NEAR:
                if(!actionTriggered && follower.getCurrentTValue() > 0.2) { intake.intake(); actionTriggered = true; }
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_NEAR);
                break;
            case WAIT_BALL_NEAR:
                if (timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_NEAR);
                break;
            case TO_SCORE_NEAR:
                if (!actionTriggered && timer.seconds() > 0.3) { intake.stop(); actionTriggered = true; }
                // ЖДЕМ ОСТАНОВКИ
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_NEAR);
                break;
            case SCORE_NEAR:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_BALL_MIDDLE);
                break;

            // 2. MIDDLE
            case TO_BALL_MIDDLE:
                if(!actionTriggered && follower.getCurrentTValue() > 0.2) { intake.intake(); actionTriggered = true; }
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_MIDDLE);
                break;
            case WAIT_BALL_MIDDLE:
                if (timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_MIDDLE);
                break;
            case TO_SCORE_MIDDLE:
                if (!actionTriggered && timer.seconds() > 0.3) { intake.stop(); actionTriggered = true; }
                // ЖДЕМ ОСТАНОВКИ
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_MIDDLE);
                break;
            case SCORE_MIDDLE:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_BALL_GATE);
                break;

            // 3. GATE (С долгим ожиданием)
            case TO_BALL_GATE:
                if(!actionTriggered && follower.getCurrentTValue() > 0.2) { intake.intake(); actionTriggered = true; }
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_GATE);
                break;
            case WAIT_BALL_GATE:
                // ЖДЕМ ДОЛЬШЕ НА ВОРОТАХ
                if (timer.seconds() > INTAKE_WAIT_GATE) setPathState(AutoState.TO_SCORE_GATE);
                break;
            case TO_SCORE_GATE:
                if (!actionTriggered && timer.seconds() > 0.3) { intake.stop(); actionTriggered = true; }
                // ЖДЕМ ОСТАНОВКИ
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_GATE);
                break;
            case SCORE_GATE:
                if (timer.seconds() > SHOOT_TIME) setPathState(AutoState.TO_BALL_FAR);
                break;

            // 4. FAR
            case TO_BALL_FAR:
                if(!actionTriggered && follower.getCurrentTValue() > 0.2) { intake.intake(); actionTriggered = true; }
                if (!follower.isBusy()) setPathStateInternal(AutoState.WAIT_BALL_FAR);
                break;
            case WAIT_BALL_FAR:
                if (timer.seconds() > INTAKE_WAIT_TIME) setPathState(AutoState.TO_SCORE_FAR);
                break;
            case TO_SCORE_FAR:
                if (!actionTriggered && timer.seconds() > 0.3) { intake.stop(); actionTriggered = true; }
                // ЖДЕМ ОСТАНОВКИ
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_FAR);
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

        turret.update(pose, vision);
        panelsTelemetry.debug("State", currentState);
        panelsTelemetry.debug("RPM", shooter.getCurrentRPM());
        panelsTelemetry.update(telemetry);
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        timer.reset();
        actionTriggered = false;

        if (newState.name().contains("SCORE") && !newState.name().contains("TO")) {
            claw.open();
            intake.intake();
        } else if (newState.name().contains("TO_BALL")) {
            claw.close();
        }

        switch (newState) {
            case TO_SCORE_PRELOAD: follower.followPath(paths.ToScorePreload, true); break;
            case TO_BALL_NEAR:    follower.followPath(paths.ToBallNear, true); break;
            case TO_SCORE_NEAR:   follower.followPath(paths.ToScoreNear, true); break;
            case TO_BALL_MIDDLE:  follower.followPath(paths.ToBallMiddle, true); break;
            case TO_SCORE_MIDDLE: follower.followPath(paths.ToScoreMiddle, true); break;
            case TO_BALL_GATE:    follower.followPath(paths.ToBallGate, true); break;
            case TO_SCORE_GATE:   follower.followPath(paths.ToScoreGate, true); break;
            case TO_BALL_FAR:     follower.followPath(paths.ToBallFar, true); break;
            case TO_SCORE_FAR:    follower.followPath(paths.ToScoreFar, true); break;
            case PARK:            follower.followPath(paths.LeaveBase, true); break;
        }
    }

    private void setPathStateInternal(AutoState state) {
        currentState = state;
        timer.reset();
        actionTriggered = false;
    }

    // === PATHS ===
    public static class Paths {
        public final Pose StartPose   = new Pose(112.4, 127.4);
        public final Pose ScorePose   = new Pose(96,84);

        public final Pose NearBallPos   = new Pose(124.5, 83);
        public final Pose MiddleBallPos = new Pose(126, 59);
        public final Pose GateBallPos   = new Pose(128.8, 59);
        public final Pose FarBallPos    = new Pose(126, 36.5);

        public final Pose ParkPose      = new Pose(117, 50);

        // HEADINGS
        double StartHeading = Math.toRadians(45);
        double ScoreHeading = Math.toRadians(-45);
        double IntakeHeading = Math.toRadians(0);
        double GateHeading = Math.toRadians(45);

        public PathChain ToScorePreload;
        public PathChain ToBallNear, ToScoreNear;
        public PathChain ToBallMiddle, ToScoreMiddle;
        public PathChain ToBallGate, ToScoreGate;
        public PathChain ToBallFar, ToScoreFar;
        public PathChain LeaveBase;

        public Paths(Follower follower) {

            // Preload
            ToScorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(StartPose, ScorePose))
                    .setLinearHeadingInterpolation(StartHeading, ScoreHeading)
                    .build();

            // 1. Near
            ToBallNear = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, NearBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToScoreNear = follower.pathBuilder()
                    .addPath(new BezierLine(NearBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            // 2. Middle
            ToBallMiddle = follower.pathBuilder()
                    .addPath(new BezierCurve(ScorePose, new Pose(98.6, 60), MiddleBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToScoreMiddle = follower.pathBuilder()
                    .addPath(new BezierLine(MiddleBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            // 3. Gate
            ToBallGate = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, GateBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, GateHeading)
                    .build();
            ToScoreGate = follower.pathBuilder()
                    .addPath(new BezierLine(GateBallPos, ScorePose))
                    .setLinearHeadingInterpolation(GateHeading, ScoreHeading)
                    .build();

            // 4. Far
            ToBallFar = follower.pathBuilder()
                    .addPath(new BezierCurve(ScorePose, new Pose(99.1, 39.5), FarBallPos))
                    .setLinearHeadingInterpolation(ScoreHeading, IntakeHeading)
                    .build();
            ToScoreFar = follower.pathBuilder()
                    .addPath(new BezierLine(FarBallPos, ScorePose))
                    .setLinearHeadingInterpolation(IntakeHeading, ScoreHeading)
                    .build();

            // Park
            LeaveBase = follower.pathBuilder()
                    .addPath(new BezierLine(ScorePose, ParkPose))
                    .setLinearHeadingInterpolation(ScoreHeading, Math.toRadians(180))
                    .build();
        }
    }
}