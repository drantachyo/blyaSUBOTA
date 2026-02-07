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

@Autonomous(name = "Pedro Auto Blue", group = "Autonomous")
@Configurable
public class PedroAutonomousBlue extends OpMode {

    // === КООРДИНАТЫ И НАСТРОЙКИ (КРАСНАЯ СТОРОНА) ===
    // X зеркалится: 144 - 138 = 6.0
    private static final double OBELISK_X = 6.0;
    private static final double OBELISK_Y = 138.0;
    // ВАЖНО: Узнай ID тега на Красной корзине! (Часто это 10, но проверь Vision)
    private static final int APRIL_TAG_ID = 10;

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

    // === ПЕРЕМЕННЫЕ ДЛЯ VARIABLE SHOOTING ===
    private double lastKnownDistance = 40.0;
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

        // СТАРТОВАЯ ПОЗИЦИЯ ЗЕРКАЛЬНАЯ
        // X: 144 - 123 = 21
        // Angle: 180 - 35 = 145
        follower.setStartingPose(new Pose(21, 124, Math.toRadians(145)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "RED SIDE READY.");
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

        // Логика стрельбы такая же, математика сама разберется с дистанцией
        double currentDist = vision.getDistanceFromTarget(APRIL_TAG_ID);
        if (currentDist != -1) {
            lastKnownDistance = currentDist;
        }

        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

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
        turret.update(pose, vision);

        // STATE MACHINE (Логика идентична синей, меняются только пути)
        switch (currentState) {
            case TO_SCORE_1:
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_1);
                break;
            case SCORE_1:
                claw.open(); intake.intake();
                if (timer.seconds() > 0.4) setPathState(AutoState.GET_SAMPLE_1);
                break;
            case GET_SAMPLE_1:
                intake.intake(); claw.open();
                if (!follower.isBusy() || follower.getCurrentTValue() > 0.95) setPathState(AutoState.TO_SCORE_2);
                break;
            case TO_SCORE_2:
                claw.close();
                if (timer.seconds() > 0.5) intake.stop();
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_2);
                break;
            case SCORE_2:
                claw.open(); intake.intake();
                if (timer.seconds() > 0.3) setPathState(AutoState.GET_SAMPLE_2);
                break;
            case GET_SAMPLE_2:
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeMiddleBalls2, true);
                    setPathState(AutoState.INTAKE_SAMPLE_2);
                }
                break;
            case INTAKE_SAMPLE_2:
                if (!follower.isBusy()) setPathState(AutoState.TO_SCORE_3);
                break;
            case TO_SCORE_3:
                claw.close();
                if(timer.seconds() > 0.5) intake.stop();
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_3);
                break;
            case SCORE_3:
                claw.open(); intake.intake();
                if (timer.seconds() > 0.3) setPathState(AutoState.GET_SAMPLE_3);
                break;
            case GET_SAMPLE_3:
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeMiddleBalls3, true);
                    setPathState(AutoState.INTAKE_SAMPLE_3);
                }
                break;
            case INTAKE_SAMPLE_3:
                if (!follower.isBusy()) setPathState(AutoState.TO_SCORE_4);
                break;
            case TO_SCORE_4:
                claw.close();
                if(timer.seconds() > 0.5) intake.stop();
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_4);
                break;
            case SCORE_4:
                claw.open(); intake.intake();
                if (timer.seconds() > 0.3) setPathState(AutoState.GET_SAMPLE_4);
                break;
            case GET_SAMPLE_4:
                intake.intake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeMiddleBalls4, true);
                    setPathState(AutoState.INTAKE_SAMPLE_4);
                }
                break;
            case INTAKE_SAMPLE_4:
                if (!follower.isBusy()) setPathState(AutoState.TO_SCORE_5);
                break;
            case TO_SCORE_5:
                claw.close();
                if(timer.seconds() > 0.5) intake.stop();
                if (!follower.isBusy() && shooter.isReady()) setPathState(AutoState.SCORE_5);
                break;
            case SCORE_5:
                claw.open(); intake.intake();
                if (timer.seconds() > 0.3) setPathState(AutoState.PARK);
                break;
            case PARK:
                hood.setBasePosition(Hood.LIMIT_BOTTOM);
                shooter.setTargetRPM(0);
                turret.idle(); intake.stop();
                if (!follower.isBusy()) currentState = AutoState.IDLE;
                break;
            case IDLE:
                break;
        }

        panelsTelemetry.debug("State", currentState);
        panelsTelemetry.debug("DistUsed", String.format("%.1f", lastKnownDistance));
        panelsTelemetry.debug("RPM Target/Curr", String.format("%.0f / %.0f", calculatedRPM, shooter.getCurrentRPM()));
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

    // === ПЕРЕСЧИТАННЫЕ ПУТИ ===
    public static class Paths {
        public PathChain StartPosition, ToScore1, MiddleBalls1, ToScore2, MiddleBalls2, IntakeMiddleBalls2;
        public PathChain ToScore3, MiddleBalls3, IntakeMiddleBalls3, ToScore4, ToMiddleBalls4, IntakeMiddleBalls4;
        public PathChain ToScore5, ToCloseBalls, ToScore6, BaseLeave;

        public Paths(Follower follower) {
            // Формулы пересчета:

            // NewX = 144 - OldX
            // NewAngle = 180 - OldAngle

            // Start: (123, 124) -> (21, 124). Angle: 35 -> 145
            StartPosition = follower.pathBuilder().addPath(new BezierLine(new Pose(21.000, 124.000), new Pose(21.000, 125.000))).setConstantHeadingInterpolation(Math.toRadians(145)).build();

            // ToScore1: 123->21, 84->60. Angle 35->145, -25->205
            ToScore1 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.000, 125.000), new Pose(60.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(205)).build();

            // MiddleBalls1: 84->60, 81.863->62.137, 129->15. Angle -25->205, 0->180
            MiddleBalls1 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60.000, 76.000), new Pose(62.137, 67.979), new Pose(15.000, 57.000))).setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(180)).build();

            // ToScore2: 129->15, 97.121->46.879, 84->60. Angle 0->180, -25->205
            ToScore2 = follower.pathBuilder().addPath(new BezierCurve(new Pose(15.000, 57.000), new Pose(46.879, 70.963), new Pose(60.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(205)).build();

            // MiddleBalls2: 84->60, 110.077->33.923, 132->12. Angle -25->205, 45->135
            MiddleBalls2 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60.000, 76.000), new Pose(33.923, 63.037), new Pose(12.000, 58.500))).setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(135)).build();

            // Intake2: 132->12. Angle 45->135
            IntakeMiddleBalls2 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 58.500), new Pose(12.000, 53.000))).setConstantHeadingInterpolation(Math.toRadians(135)).build();

            // ToScore3: 132->12, 84->60. Angle 45->135, -25->205
            ToScore3 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 53.000), new Pose(60.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(205)).build();

            // MiddleBalls3: 84->60, 132->12. Angle -25->205, 45->135
            MiddleBalls3 = follower.pathBuilder().addPath(new BezierLine(new Pose(60.000, 76.000), new Pose(12.000, 58.500))).setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(135)).build();

            // Intake3: 132->12. Angle 45->135
            IntakeMiddleBalls3 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 58.500), new Pose(12.000, 53.000))).setConstantHeadingInterpolation(Math.toRadians(135)).build();

            // ToScore4: 132->12, 84->60. Angle 45->135, -25->205
            ToScore4 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 53.000), new Pose(60.000, 76.000))).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(205)).build();

            // ToMiddle4: 84->60, 132->12. Angle -25->205, 45->135
            ToMiddleBalls4 = follower.pathBuilder().addPath(new BezierLine(new Pose(60.000, 76.000), new Pose(12.000, 58.500))).setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(135)).build();

            // Intake4: 132->12. Angle 45->135
            IntakeMiddleBalls4 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 58.500), new Pose(12.000, 53.000))).setConstantHeadingInterpolation(Math.toRadians(135)).build();

            // ToScore5: 132->12, 90->54. Angle 45->135, -25->205
            ToScore5 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 53.000), new Pose(54.000, 83.000))).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(205)).build();

            // ToClose: 90->54, 128->16. Tangent auto
            ToCloseBalls = follower.pathBuilder().addPath(new BezierLine(new Pose(54.000, 83.000), new Pose(16.000, 83.000))).setTangentHeadingInterpolation().build();

            // ToScore6: 128->16, 90->54. Tangent auto, reversed
            ToScore6 = follower.pathBuilder().addPath(new BezierLine(new Pose(16.000, 83.000), new Pose(54.000, 83.000))).setTangentHeadingInterpolation().setReversed().build();

            // BaseLeave: 90->54, 117->27. Angle 0->180, 180->0
            BaseLeave = follower.pathBuilder().addPath(new BezierLine(new Pose(54.000, 83.000), new Pose(27.000, 72.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)).build();
        }
    }
}