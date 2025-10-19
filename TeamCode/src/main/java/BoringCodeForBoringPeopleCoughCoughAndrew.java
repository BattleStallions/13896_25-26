import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.net.CacheRequest;


@TeleOp
public class BoringCodeForBoringPeopleCoughCoughAndrew extends LinearOpMode {
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor motorCore;
    CRServo servoRight;
    CRServo servoLeft;


//    BNO055IMU imu;

    private IMU imu;

    // Put custom functions/variables here.




//    public void drive(double forward, double strafe, double rotate) {
//        double frontLeftPower = forward + strafe + rotate;
//        double backLeftPower = forward - strafe + rotate;
//
//        double frontRightPower = forward - strafe - rotate;
//        double backRightPower = forward + strafe - rotate;
//
//        double maxPower = 1.0;
//        double maxSpeed = 1.0;
//
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
//        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
//        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
//        backRight.setPower(maxSpeed * (backRightPower / maxPower));
//    }

//    public void driveFieldRelative(double forward, double strafe, double rotate){
//        double theta = Math.atan2(forward, strafe);
//        double r = Math.hypot(strafe, forward);
//
//        theta = AngleUnit.normalizeRadians(theta -
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//
//        double newForward = r * Math.sin(theta);
//        double newStrafe = r * Math.cos(theta);
//
//        this.drive(newForward, newStrafe, rotate);
//    }


    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");
        motorCore = hardwareMap.get(DcMotor.class, "motorCore");
//        ColorSensor color1 = hardwareMap.get(ColorSensor.class, "color1");
//        DistanceSensor distance1 = hardwareMap.get(DistanceSensor.class, "distance1");

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        AprilTagDetection detection;
        waitForStart();
        while (opModeIsActive()){
            backLeft.setPower(2*(gamepad1.right_trigger - gamepad1.left_trigger));
            backRight.setPower(2*(gamepad1.right_trigger - gamepad1.left_trigger));
            frontLeft.setPower(2*(gamepad1.right_trigger - gamepad1.left_trigger));
            frontRight.setPower(2*(gamepad1.right_trigger - gamepad1.left_trigger));

            backLeft.setPower(gamepad1.left_stick_x);
            backRight.setPower(-gamepad1.left_stick_x);
            frontLeft.setPower(gamepad1.left_stick_x);
            frontRight.setPower(-gamepad1.left_stick_x);
            }
            if (gamepad1.right_bumper){
                backLeft.setPower(1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(1);
            }
            if (gamepad1.left_bumper){
                backLeft.setPower(-1);
                backRight.setPower(1);
                frontLeft.setPower(1);
                frontRight.setPower(-1);
            }




            motorCore.setPower(gamepad1.right_stick_y);
            servoLeft.setPower(gamepad1.right_stick_y);
            servoRight.setPower(gamepad1.right_stick_y);
        }


    }


