package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("All")

public class Brickbot {         //TODO: Implement threads
	/* Make Singleton */
	private static Brickbot robot = new Brickbot();

	private Brickbot() {}

	public static Brickbot getInstance() {
		return robot;
	}

	public Telemetry telemetry;
	public DigitalChannel digIn;

	/* Gyro */
	public ModernRoboticsI2cGyro gyro;

	/* Motors */
	public DcMotor motorFrontLeft;
	public DcMotor motorFrontRight;
	public DcMotor motorBackLeft;
	public DcMotor motorBackRight;
	public DcMotor motorRotation;
	public DcMotor motorExtension;
	public DcMotor motorLift;

	/* Servos */
	public CRServo crServo;
	public Servo servoWall;
	public Servo servoBox;

	/* Senzori */
	public DigitalChannel limitSwitchSus;
	public DigitalChannel limitSwitchJos;

	/* Members */
	//TODO: Verify ore coords
	public      OpenGLMatrix            LEFT_ORE_COORDS     = OpenGLMatrix.translation((float) DistanceUnit.INCH.toMm(24.5d), (float) DistanceUnit.INCH.toMm(46.0d), 0);
	public      OpenGLMatrix            RIGHT_ORE_COORDS    = OpenGLMatrix.translation((float) DistanceUnit.INCH.toMm(46), (float) DistanceUnit.INCH.toMm(24.5d), 0);
	public      OpenGLMatrix            CENTER_ORE_COORDS   = OpenGLMatrix.translation((float) DistanceUnit.INCH.toMm(35.25d), (float) DistanceUnit.INCH.toMm(35.25d), 0);

	public      OpenGLMatrix            coords              = null;

	private     Drive                   drive               = new Drive();
	private     Drive.Direction         Direction           = null;

	private     Collector               collector           = new Collector();

	private     Lift                    lift                = new Lift();

	protected   DistanceUnit            distanceUnit        = DistanceUnit.CM;
	protected   AngleUnit               angleUnit           = AngleUnit.DEGREES;

	public      TFOreIdentification     tfID                = new TFOreIdentification();

	/* Initialize */
	public void init(Telemetry telemetry, HardwareMap hwMap) {
		this.telemetry = telemetry;

		gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

		motorFrontLeft = hwMap.get(DcMotor.class, "motorfl");
		motorFrontRight = hwMap.get(DcMotor.class, "motorfr");
		motorBackLeft = hwMap.get(DcMotor.class, "motorbl");
		motorBackRight = hwMap.get(DcMotor.class, "motorbr");
		motorRotation = hwMap.get(DcMotor.class, "motorRotation");
		motorExtension = hwMap.get(DcMotor.class, "motorExtension");
		motorLift = hwMap.get(DcMotor.class, "motorLift");

		servoBox = hwMap.servo.get("servobox");
		servoWall = hwMap.servo.get("servowall");
		crServo = hwMap.crservo.get("crservo");

		limitSwitchJos=hwMap.get(DigitalChannel.class,"LimitSensorJos");
		limitSwitchSus=hwMap.get(DigitalChannel.class,"LimitSensorSus");

		telemetry.addData(">", "Calibrating Gyro");
		telemetry.update();

		//gyro.calibrate();

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

		//while (gyro.isCalibrating()) {}

		motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		servoWall.setPosition(0);

		//gyro.resetZAxisIntegrator();

		//tfID.init(hwMap);
/*
		coords = OpenGLMatrix.translation(0, 0, 0)      //TODO: Find robot start position
				.multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 45));
*/
		telemetry.addData(">", "Robot Ready.");
		telemetry.update();
	}

	public void sampleGoldOre() {
		tfID.activate();
		String position = tfID.getGoldOrePosition();
		tfID.shutdown();

		tfID.dropWall();

		drive.move(Direction.RIGHT, distanceUnit, 40);

		switch (position) {
			case "RIGHT":
				drive.move(Direction.BACKWARD, distanceUnit, 50);
				break;
			case "LEFT":
				drive.move(Direction.FORWARD, distanceUnit, 50);
				break;
		}

		drive.move(Direction.RIGHT, distanceUnit, 55);

		tfID.raiseWall();
	}

	public void test() {
		telemetry.clear();
	}
}
