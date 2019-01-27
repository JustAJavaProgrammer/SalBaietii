package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.TFOreIdentification;
import org.firstinspires.ftc.teamcode.robot.Lift;

@SuppressWarnings("All")

public class HardwareRO036 {
    public ModernRoboticsI2cGyro gyro= null;
    Telemetry telemetry;

    /* Motors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorRotation = null;
    public DcMotor motorExtension = null;
    public DcMotor motorLift = null;

    /* Servos */
    public CRServo crServo = null;
    public Servo servoWall = null;
    public Servo servoBox = null;

    /* Senzori */
    DigitalChannel limitSwitchJos;
    DigitalChannel limitSwitchSus;

    /* Members */
    //TODO: Verify ore coords
    public OpenGLMatrix LEFT_ORE_COORDS     = OpenGLMatrix.translation((float) DistanceUnit.INCH.toMm(24.5d), (float) DistanceUnit.INCH.toMm(46.0d), 0);
    public OpenGLMatrix RIGHT_ORE_COORDS    = OpenGLMatrix.translation((float) DistanceUnit.INCH.toMm(46), (float) DistanceUnit.INCH.toMm(24.5d), 0);
    public OpenGLMatrix CENTER_ORE_COORDS   = OpenGLMatrix.translation((float) DistanceUnit.INCH.toMm(35.25d), (float) DistanceUnit.INCH.toMm(35.25d), 0);

    public OpenGLMatrix coords = null;

    private Drive drive               = new Drive();
    private Drive.Direction Direction = null;

    private Collector collector = new Collector();

    private Lift lift = new Lift();

    protected DistanceUnit distanceUnit = DistanceUnit.CM;
    protected AngleUnit angleUnit = AngleUnit.DEGREES;

    public TFOreIdentification tfID = new TFOreIdentification();
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRO036(){

    }
    public void init(HardwareMap ahwMap){
        hwMap=ahwMap;
        motorFrontLeft = hwMap.get(DcMotor.class, "motorfl");
        motorFrontRight = hwMap.get(DcMotor.class, "motorfr");
        motorBackLeft = hwMap.get(DcMotor.class, "motorbl");
        motorBackRight = hwMap.get(DcMotor.class, "motorbr");
        motorRotation = hwMap.get(DcMotor.class, "motorRotation");
        motorExtension = hwMap.get(DcMotor.class, "motorExtension");
        motorLift = hwMap.get(DcMotor.class, "motorLift");

        limitSwitchJos=hwMap.get(DigitalChannel.class,"LimitSensorJos");
        limitSwitchSus=hwMap.get(DigitalChannel.class,"LimitSensorSus");

        servoBox = hwMap.get(Servo.class, "servobox");
        servoWall = hwMap.get(Servo.class,"servowall");
        crServo = hwMap.get(CRServo.class,"crservo");
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtension.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);

        crServo.setDirection(CRServo.Direction.REVERSE);
        servoWall.setDirection(Servo.Direction.REVERSE);
        servoBox.setDirection(Servo.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoWall.setPosition(0);
        servoBox.setPosition(0.1);

    }
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }



}
