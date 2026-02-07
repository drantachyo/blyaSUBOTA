package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Claw {
    private final Servo servo;

    // Константы позиций для Bylyzar
    public static double OPEN_POSITION = 0.554;
    public static double CLOSE_POSITION = 0.714;

    public Claw(HardwareMap hw) {
        servo = hw.get(Servo.class, "claw");
    }

    public void open() {
        servo.setPosition(OPEN_POSITION);
    }

    public void close() {
        servo.setPosition(CLOSE_POSITION);
    }

    // Метод специально для теста и тюнинга
    public void setRawPosition(double pos) {
        servo.setPosition(pos);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}