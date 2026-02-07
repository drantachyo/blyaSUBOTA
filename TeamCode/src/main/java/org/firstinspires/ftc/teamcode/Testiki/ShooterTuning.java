package org.firstinspires.ftc.teamcode.Testiki;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name = "🔧 Shooter Tuning (+Feed)", group = "Tuning")
public class ShooterTuning extends OpMode {

    private Shooter shooter;
    private Intake intake;
    private Claw claw;

    // Меняй эту переменную в ByLazar Panel
    public static double TEST_RPM = 0;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

        claw.close(); // На старте закрываем

        telemetry.addLine("READY TO TUNE.");
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("[A] (Hold) - Run Intake");
        telemetry.addLine("[LB] - Open Claw");
        telemetry.addLine("[RB] - Close Claw");
    }

    @Override
    public void loop() {
        // 1. Обновляем коэффициенты и скорость
        shooter.update();
        shooter.setTargetRPM(TEST_RPM);

        // 2. Логика Интейка (Зажми A, чтобы подать кольцо)
        if (gamepad1.a) {
            intake.intake();
        } else {
            intake.stop();
        }

        // 3. Логика Клешни (LB - Открыть, RB - Закрыть)
        if (gamepad1.left_bumper) {
            claw.open();
        }
        if (gamepad1.right_bumper) {
            claw.close();
        }

        // 4. Телеметрия
        double currentRPM = shooter.getCurrentRPM();

        telemetry.addData("TARGET", TEST_RPM);
        telemetry.addData("ACTUAL", "%.0f", currentRPM);
        telemetry.addData("ERROR", "%.0f", TEST_RPM - currentRPM);

        telemetry.addData("--- PIDF ---", "");
        telemetry.addData("P", Shooter.kP);
        telemetry.addData("F", Shooter.kF);

        telemetry.update();
    }
}