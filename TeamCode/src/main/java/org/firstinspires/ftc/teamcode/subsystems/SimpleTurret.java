package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime; // <--- ДОБАВЛЕНО
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

    public static double TICKS_PER_RADIAN = 191.0;
    public static double LIMIT_MIN = Math.toRadians(-120);
    public static double LIMIT_MAX = Math.toRadians(120);

    // === PID ===
    public static double ODO_P = 1;
    public static double ODO_I = 0.0;
    public static double ODO_D = 0.08;



    public static double VIS_P = 1.38;
    public static double VIS_I = 0.0;
    public static double VIS_D = 0.09;

    public static double kS = 0.02;

    // === НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ ТАЙМАУТА ===
    private ElapsedTime timeSinceLastSeen = new ElapsedTime(); // Таймер
    public static double LOCK_TIMEOUT = 1; // Время ожидания перед сбросом (сек)

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

    public void setTargetAngle(double angleRad) {
        this.currentState = State.MANUAL;
        this.targetAngle = angleRad;
        this.targetTagId = -1;
    }

    public void idle() {
        this.currentState = State.IDLE;
        this.targetTagId = -1;
    }

    public void update(Pose robotPose, Vision vision) {
        switch (currentState) {
            case IDLE:
                controller.setPIDF(ODO_P, ODO_I, ODO_D, 0, kS);
                break;

            case MANUAL:
                controller.setPIDF(ODO_P, ODO_I, ODO_D, 0, kS);
                break;

            case SEARCHING:
                controller.setPIDF(ODO_P, ODO_I, ODO_D, 0, kS);

                if (robotPose != null) {
                    calculateOdometryAngle(robotPose);
                }

                // Если увидели тег -> переходим в LOCKED
                if (vision != null && vision.getTarget(targetTagId) != null) {
                    currentState = State.LOCKED;
                    timeSinceLastSeen.reset(); // <--- Сбрасываем таймер при входе
                }
                break;

            case LOCKED:
                controller.setPIDF(VIS_P, VIS_I, VIS_D, 0, kS);

                AprilTagDetection tag = (vision != null) ? vision.getTarget(targetTagId) : null;

                if (tag != null) {
                    // Видим тег: сбрасываем таймер и обновляем угол
                    timeSinceLastSeen.reset();

                    double bearingRad = Math.toRadians(tag.ftcPose.bearing);
                    double currentRad = getCurrentAngle();
                    targetAngle = currentRad + bearingRad;
                } else {
                    // Тег потерян: проверяем, сколько времени прошло
                    if (timeSinceLastSeen.seconds() > LOCK_TIMEOUT) {
                        // Тайм-аут вышел -> переходим на одометрию
                        currentState = State.SEARCHING;
                    }
                    // Если тайм-аут еще не вышел, мы ничего не меняем.
                    // targetAngle остается старым (держим последнюю позицию).
                }
                break;
        }

        // ИСПОЛНЕНИЕ
        targetAngle = Range.clip(targetAngle, LIMIT_MIN, LIMIT_MAX);
        double currentAngle = getCurrentAngle();
        double error = MathUtils.normalizeAngle(targetAngle - currentAngle);

        double power = controller.calculate(error);

        if (currentAngle >= LIMIT_MAX && power > 0) power = 0;
        if (currentAngle <= LIMIT_MIN && power < 0) power = 0;

        motor.setPower(power);
    }

    private void calculateOdometryAngle(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double globalAngle = Math.atan2(dy, dx);
        double relativeAngle = globalAngle - pose.getHeading();
        targetAngle = MathUtils.normalizeAngle(relativeAngle);
    }

    public double getCurrentAngle() {
        return motor.getCurrentPosition() / TICKS_PER_RADIAN;
    }

    public String getState() {
        return currentState.toString();
    }
}