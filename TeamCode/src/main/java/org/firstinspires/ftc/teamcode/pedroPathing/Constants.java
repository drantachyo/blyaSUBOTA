package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-31.71)
            .lateralZeroPowerAcceleration(-63.46)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.012, 0.025))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.09,0,0.01,0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.082, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.3,0,0.09,0.023))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0.0,0.0026,0.6,0.057))
            .useSecondaryDrivePIDF(false)
            .mass(10.75);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(86.76)
            .yVelocity(71.68);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("lf")
            .strafeEncoder_HardwareMapName("rr")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardPodY(2.7559055118)
            .strafePodX(-5.4330708661)
            .forwardTicksToInches(0.001989436789)
            .strafeTicksToInches(0.001989436789)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardEncoderDirection(Encoder.FORWARD);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 0.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
