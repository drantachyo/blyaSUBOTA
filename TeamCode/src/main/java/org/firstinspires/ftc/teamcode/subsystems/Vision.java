package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    public Vision(HardwareMap hw) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // .setNumThreads(4) // Убрал, т.к. по дефолту он сам выбирает оптимальное, но 4 тоже ок
                .build();

        // Decimation (прореживание) - ускоряет обработку за счет снижения качества
        // 3 - отличный баланс для дистанций до 2-3 метров
        aprilTag.setDecimation(3);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1")) // Проверь имя в конфиге!
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                // ВАЖНО: Отключаем вывод на экран телефона для скорости!
                // Если нужно отлаживать - закомментируй эту строку.
                .enableLiveView(false)
                .setAutoStopLiveView(true)
                .build();
    }

    /**
     * Пытается применить настройки для движения.
     * Возвращает true, если успешно применилось (камера готова).
     */
    public boolean applyCombatSettings() {
        // Выдержка 2мс, Усиление 250 (можно поиграть с Gain, если слишком темно)
        return setManualExposure(2, 250);
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        // Если камера еще не стримит - выходим, чтобы не крашнуть или не зависнуть
        if (portal == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        // ВАЖНО: Мы не хотим спать (Thread.sleep) в основном потоке Loop!
        // Но так как настройки применяются один раз в Init_Loop, здесь это допустимо.
        try {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);

            // Переключаем в Manual, если еще не там
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }

            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);

            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);

            return true;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        } catch (Exception e) {
            // Иногда камера кидает исключения, если контроль недоступен
            return false;
        }
    }

    public AprilTagDetection getTarget(int id) {
        // Защита от NullPointerException
        if (aprilTag == null) return null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return null;

        for (AprilTagDetection d : detections) {
            if (d.id == id && d.metadata != null) return d;
        }
        return null;
    }

    public double getDistanceFromTarget(int id) {
        AprilTagDetection tag = getTarget(id);
        if (tag != null) {
            return tag.ftcPose.range;
        }
        return -1.0;
    }

    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }
}