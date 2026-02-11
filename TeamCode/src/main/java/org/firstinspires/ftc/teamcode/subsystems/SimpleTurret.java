package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.math.MathUtils;
import org.firstinspires.ftc.teamcode.math.PIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
public class SimpleTurret {

    public double getTargetAngle() {
        return targetAngle;
    }

    public enum State { IDLE, SEARCHING, LOCKED, MANUAL }
    private State currentState = State.IDLE;

    private final DcMotorEx motor;
    private final PIDFController controller;

    // === НАСТРОЙКИ ===
    public static double TICKS_PER_RADIAN = 191.0;

    // ЗАЩИТА ПРОВОДОВ:
    public static double WIRE_LIMIT = Math.toRadians(175);

    // === PID ===
    public static double ODO_P = 1.0;
    public static double ODO_I = 0.0;
    public static double ODO_D = 0.08;

    public static double VIS_P = 1.38;
    public static double VIS_I = 0.0;
    public static double VIS_D = 0.09;

    public static double kS = 0.02;


    private ElapsedTime timeSinceLastSeen = new ElapsedTime();
    public static double LOCK_TIMEOUT = 1.5;

    // НОВОЕ: Дедбенд 2 градуса
    public static double VISION_DEADBAND = Math.toRadians(2.0);

    private double targetAngle = 0;
    private int targetTagId = -1;
    private double targetX = 0, targetY = 0;

    public SimpleTurret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        motor.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(ODO_P, ODO_I, ODO_D, 0, kS);
    }

    public void track(int id, double x, double y) {
        this.targetTagId = id;
        this.targetX = x;
        this.targetY = y;
        if (currentState == State.IDLE || currentState == State.MANUAL) {
            currentState = State.SEARCHING;
        }
    }

    public void hold() {
        this.targetAngle = getCurrentAngle();
        this.currentState = State.MANUAL;
    }

    public void setTargetAngle(double angleRad) {
        this.currentState = State.MANUAL;
        this.targetAngle = angleRad;
    }

    public void idle() {
        this.currentState = State.IDLE;
        motor.setPower(0);
    }

    public void update(Pose robotPose, Vision vision) {
        // --- 1. ЛОГИКА ВЫБОРА ЦЕЛИ (State Machine) ---
        switch (currentState) {
            case IDLE:
                motor.setPower(0);
                return;

            case MANUAL:
                controller.setPIDF(ODO_P, ODO_I, ODO_D, 0, kS);
                break;

            case SEARCHING:
                controller.setPIDF(ODO_P, ODO_I, ODO_D, 0, kS);
                if (robotPose != null) calculateOdometryAngle(robotPose);

                if (vision != null && vision.getTarget(targetTagId) != null) {
                    currentState = State.LOCKED;
                    timeSinceLastSeen.reset();
                }
                break;

            case LOCKED:
                controller.setPIDF(VIS_P, VIS_I, VIS_D, 0, kS);
                AprilTagDetection tag = (vision != null) ? vision.getTarget(targetTagId) : null;

                if (tag != null) {
                    timeSinceLastSeen.reset();

                    // Обновляем угол только если ошибка больше дедбенда
                    double bearingRad = Math.toRadians(tag.ftcPose.bearing);
                    if (Math.abs(bearingRad) > VISION_DEADBAND) {
                        targetAngle = getCurrentAngle() + bearingRad;
                    }

                } else {
                    if (timeSinceLastSeen.seconds() > LOCK_TIMEOUT) {
                        currentState = State.SEARCHING;
                    }
                }
                break;
        }

        // --- 2. ЛОГИКА ЗАЩИТЫ ПРОВОДОВ (Wire Protection) ---
        double currentNorm = MathUtils.normalizeAngle(getCurrentAngle());
        double targetNorm = MathUtils.normalizeAngle(targetAngle);
        double safeTarget = Range.clip(targetNorm, -WIRE_LIMIT, WIRE_LIMIT);
        double error = safeTarget - currentNorm;

        // Если ошибка мала - ставим 0 мощность (анти-дрожание)
        if (currentState == State.LOCKED && Math.abs(error) < VISION_DEADBAND) {
            motor.setPower(0);
            return;
        }

        double power = controller.calculate(error);
        motor.setPower(power);
    }

    private void calculateOdometryAngle(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double globalAngle = Math.atan2(dy, dx);
        double relativeAngle = globalAngle - pose.getHeading();
        this.targetAngle = relativeAngle;
    }

    public double getCurrentAngle() {
        return motor.getCurrentPosition() / TICKS_PER_RADIAN;
    }
}