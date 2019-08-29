package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "GyroTest", group = "8872")
//@Disabled
public class GyroTest extends LinearOpMode {
    Chassis robot = new Chassis();
    Orientation angles;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        composeTelemetry();
        waitForStart();
        gyroTurn(90);

        stopRobot();


    }

    public void gyroTurn(double targetAngle) {

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double headingAngle = angles.firstAngle;

        while (headingAngle < targetAngle && opModeIsActive()) {
            robot.leftFront.setPower(-0.2);
            robot.rightFront.setPower(0.2);
            robot.leftRear.setPower(-0.2);
            robot.rightRear.setPower(0.2);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = angles.firstAngle;
            telemetry.update();
        }

        stopRobot();
        sleep(10000);

    }

    public void stopRobot() {
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
    }

    public void composeTelemetry() {
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
