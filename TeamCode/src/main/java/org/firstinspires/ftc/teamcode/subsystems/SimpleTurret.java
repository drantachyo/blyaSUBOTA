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

    public enum State { IDLE, SEARCHING, LOCKED, MANUAL }
    private State currentState = State.IDLE;

    private final DcMotorEx motor;
    private final PIDFController controller;

    // === НАСТРОЙКИ ===
    public static double TICKS_PER_RADIAN = 191.0;

    // ЗАЩИТА ПРОВОДОВ:
    // Ставим лимит чуть меньше 180 (например, 175 градусов / 3.05 рад).
    // Это гарантирует, что турель никогда не поедет в "мертвую зону" сзади.
    public static double WIRE_LIMIT = Math.toRadians(175);

    // === PID ===
    public static double ODO_P = 1;
    public static double ODO_I = 0.0;
    public static double ODO_D = 0.08;

    public static double VIS_P = 1.2;
    public static double VIS_I = 0.0;
    public static double VIS_D = 0.9;

    public static double kS = 0.02; // Компенсация трения

    private ElapsedTime timeSinceLastSeen = new ElapsedTime();
    public static double LOCK_TIMEOUT = 1.5;

    private double targetAngle = 0; // Целевой угол
    private int targetTagId = -1;
    private double targetX = 0, targetY = 0;

    public SimpleTurret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        motor.setDirection(DcMotor.Direction.REVERSE);

        // ВАЖНО: При инициализации считаем, что башня смотрит ПРЯМО (0 градусов).
        // Выровняйте её руками перед стартом!
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
                    // Считаем абсолютный угол цели
                    double bearingRad = Math.toRadians(tag.ftcPose.bearing);
                    targetAngle = getCurrentAngle() + bearingRad;
                } else {
                    if (timeSinceLastSeen.seconds() > LOCK_TIMEOUT) {
                        currentState = State.SEARCHING;
                    }
                }
                break;
        }

        // --- 2. ЛОГИКА ЗАЩИТЫ ПРОВОДОВ (Wire Protection) ---

        // Шаг А: Приводим все углы к диапазону -180...+180.
        // Это важно, чтобы мы понимали, где лево, а где право.
        double currentNorm = MathUtils.normalizeAngle(getCurrentAngle());
        double targetNorm = MathUtils.normalizeAngle(targetAngle);

        // Шаг Б: Жестко ограничиваем цель (Clip).
        // Если математика хочет -179, а провода пускают только до -175 -> ставим -175.
        // Это "Стена".
        double safeTarget = Range.clip(targetNorm, -WIRE_LIMIT, WIRE_LIMIT);

        // Шаг В: Считаем ошибку ОБЫЧНЫМ вычитанием.
        // Мы НЕ используем здесь normalizeAngle(error)!
        // Если цель -170, а мы на +170 -> ошибка будет -340.
        // Робот поймет: "Ого, надо крутиться назад на полный оборот через 0".
        // Провода спасены.
        double error = safeTarget - currentNorm;

        // Отправляем в PID
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