package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("ALL")

public class Robot {
    public static Telemetry telemetry;

    public static DcMotor motorFrontLeft;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackLeft;
    public static DcMotor motorBackRight;
    public static DcMotor motorRotation;
    public static DcMotor motorExtension;
    public static DcMotor motorLift;

    public static Servo servoBox;
    public static Servo servoWall;
    public static CRServo crServo;

    public static TouchSensor liftMax;
    public static TouchSensor liftMin;

    public static void init(HardwareMap hwMap, Telemetry telemetry) {

        //gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        Robot.telemetry           = telemetry;

        Robot.liftMax             = hwMap.touchSensor.get("liftMax");
        Robot.liftMin             = hwMap.touchSensor.get("liftMin");

        Robot.motorFrontLeft      = hwMap.dcMotor.get("motorfl");
        Robot.motorFrontRight     = hwMap.dcMotor.get("motorfr");
        Robot.motorBackLeft       = hwMap.dcMotor.get("motorbl");
        Robot.motorBackRight      = hwMap.dcMotor.get("motorbr");
        Robot.motorRotation       = hwMap.dcMotor.get("motorRotation");
        Robot.motorExtension      = hwMap.dcMotor.get("motorExtension");
        Robot.motorLift           = hwMap.dcMotor.get("motorLift");

        Robot.servoBox = hwMap.servo.get("servobox");
        Robot.servoWall = hwMap.servo.get("servowall");
        Robot.crServo = hwMap.crservo.get("crservo");

        //gyro.calibrate();

        Robot.motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Robot.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Robot.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.motorRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.motorExtension.setDirection(DcMotorSimple.Direction.FORWARD);
        Robot.motorLift.setDirection(DcMotorSimple.Direction.FORWARD);

        Robot.crServo.setDirection(CRServo.Direction.REVERSE);
        Robot.servoWall.setDirection(Servo.Direction.REVERSE);
        Robot.servoBox.setDirection(Servo.Direction.REVERSE);

        Robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.motorRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.motorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //while (gyro.isCalibrating()) {}

        Robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Robot.servoWall.setPosition(0);

        //gyro.resetZAxisIntegrator();

        //tfID.init(hwMap);
        /*
		coords = OpenGLMatrix.translation(0, 0, 0)      //TODO: Find robot start position
				.multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 45));
        */
    }



}
