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

@TeleOp(name = "BLUE TELEOP ULTIMATE", group = "Final")
public class FinalCombinedTeleOpBlue extends OpMode {

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

    // === КООРДИНАТЫ И НАСТРОЙКИ (СИНИЕ) ===
    private static final double TARGET_X = 6;
    private static final double TARGET_Y = 138;
    private static final int TAG_ID = 20;

    private static final Pose START_POSE = new Pose(133.4, 8, Math.toRadians(0));

    private Pose savedAutoPose = null;

    public enum RobotState { IDLE, INTAKE, OUTTAKE, PREP_SHOOT, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    // === ПЕРЕМЕННЫЕ ДЛЯ СТРЕЛЬБЫ ===
    private double lastKnownDistance = 40.0;
    private double calculatedRPM = 0;
    private double calculatedHood = 0.3;

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
            follower.setStartingPose(new Pose(73.5, 8, Math.toRadians(0)));
            telemetry.addLine("⚠️ NO AUTO POSE. USING DEFAULT (BLUE).");
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
        telemetry.addData("Side", "BLUE");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Pose p = follower.getPose();
        telemetry.addData("Waiting Start", "X:%.1f Y:%.1f H:%.1f",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        if (savedAutoPose != null) {
            follower.setPose(savedAutoPose);
        }

        vision.applyCombatSettings();
    }

    @Override
    public void loop() {
        // 1. ОБНОВЛЕНИЕ СИСТЕМ
        follower.update();
        shooter.update();
        Pose currentPose = follower.getPose();

        // 2. РАСЧЕТ ДИСТАНЦИИ
        double dist = vision.getDistanceFromTarget(TAG_ID);
        boolean tagVisible = (dist != -1);

        if (tagVisible) {
            lastKnownDistance = dist;
        }

        calculatedRPM = ShooterMath.calculateRPM(lastKnownDistance);
        calculatedHood = ShooterMath.calculateHood(lastKnownDistance);

        // 3. СБРОС КООРДИНАТ
        if (gamepad1.options) {
            follower.setPose(START_POSE);
        }

        // 4. УПРАВЛЕНИЕ ШАССИ + ТОРМОЗ
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
                    gamepad1.left_stick_y * speedMultiplier,
                    gamepad1.left_stick_x * speedMultiplier,
                    turn * 0.5,
                    false
            );
        }

        // 5. STATE MACHINE
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

        // 6. УПРАВЛЕНИЕ ТУРЕЛЬЮ
        if (currentState == RobotState.PREP_SHOOT || currentState == RobotState.SHOOTING) {
            turret.track(TAG_ID, TARGET_X, TARGET_Y);
        } else {
            turret.idle();
        }

        turret.update(currentPose, vision);

        // 7. ТЕЛЕМЕТРИЯ
        telemetry.addData("═══════════════════════", "");
        telemetry.addData("SIDE", "BLUE");
        telemetry.addData("STATE", currentState);
        telemetry.addData("═══════════════════════", "");

        // VISION STATUS
        if (tagVisible) {
            telemetry.addData("📷 CAMERA", "✅ TAG VISIBLE");
            telemetry.addData("Distance", "%.1f inch", dist);
        } else {
            telemetry.addData("📷 CAMERA", "❌ TAG LOST");
            telemetry.addData("Last Dist", "%.1f inch", lastKnownDistance);
        }

        telemetry.addData("═══════════════════════", "");
        telemetry.addData("Pose", "X:%.1f Y:%.1f H:%.1f",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Calc RPM", "%.0f", calculatedRPM);
        telemetry.addData("Calc Hood", "%.3f", calculatedHood);
        telemetry.update();
    }
}