package org.firstinspires.ftc.teamcode.Testiki;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.math.PIDFController; // Твой класс
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "👁️ VISION PID TUNER (CLEAN)", group = "Test")
public class PIDtuningVISION extends OpMode {

    // === КРУТИ ЭТИ ЦИФРЫ В DASHBOARD ===
    public static double p = 0.02;
    public static double i = 0.0;
    public static double d = 0.001;
    public static double f = 0.0;

    // === ОБОРУДОВАНИЕ ===
    private DcMotorEx turretMotor;
    private Vision vision;
    private PIDFController controller; // Твой контроллер

    private static final int TAG_ID = 24;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        vision = new Vision(hardwareMap);

        // Инициализируем твой контроллер
        controller = new PIDFController(p, i, d, f, 0.0);

        telemetry.addLine("READY. Open Dashboard.");
        telemetry.addLine("Hold GP1 [A] to enable tracking.");
    }

    @Override
    public void init_loop() {
        vision.applyCombatSettings();
    }

    @Override
    public void loop() {
        // Обновляем коэффициенты на лету (чтобы работало из Dashboard)
        controller.setPIDF(p, i, d, f, 0.0);

        AprilTagDetection tag = vision.getTarget(TAG_ID);

        if (gamepad1.a && tag != null) {

            // Цель: bearing должен быть 0
            // bearing - это угол на тег. Если тег слева (+), bearing > 0.
            // Нам нужно повернуть влево (-), чтобы уменьшить угол.
            // Поэтому error = 0 - bearing = -bearing.
            double error = -tag.ftcPose.bearing;

            // Считаем через твой класс
            double power = controller.calculate(error);

            turretMotor.setPower(power);

            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", power);

        } else {
            turretMotor.setPower(0);
            // Если нужно сбрасывать интеграл при отпускании кнопки,
            // добавь метод reset() в свой PIDFController, если его там нет.
            // controller.reset();
            telemetry.addLine("⏸️ IDLE");
        }

        telemetry.addData("Tag Visible?", tag != null ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}