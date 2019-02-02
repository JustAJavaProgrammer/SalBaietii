package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@SuppressWarnings("all")
public class Sample {
	private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
	private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
	private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

	private static final String VUFORIA_KEY = "AW+QDt3/////AAABmUOonI8rb0LTkFhL3nkIkY+ORnThf7+Fzmt4rOxdKkealCvQLhEIRrrk3YmbsfoMzTIzo1sHEcMOdubdxhR7QJxAivwT80q3G0w6zH5uYSDN1obUzkJbAcmgrD19wnSbpJIS1q152Vc1Ao/FdnSEqGBux9dM5Ng2N8ADBWtO0HzMk2iIx6z3MFGa5xV49nlxl0PhFUvcElWABhz4ToI4c3RTJyge0FcNiUfz/yqkjmM6dgqhC7gbtEoLyUncltUsWzLC4r51ebsBj+oSNE4K8nIzFiaWS+AZa3zz/ORjd8RVevLV261raejwbBQGwmkCHBIRuyZvdJdVU3y3YF6iaAx5j45V04amDLer3FeXzJGd";
	private static VuforiaLocalizer vuforia;
	private static TFObjectDetector tfod;

	/* INITIALIZE */
	public static void init(HardwareMap hwMap) {
		initVuforia(hwMap);

		if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
			initTfod(hwMap);
		} else {
			Robot.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
			Robot.telemetry.update();

			throw new NullPointerException();
		}
	}

	private static void initVuforia(HardwareMap hwMap) {
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

		vuforia = ClassFactory.getInstance().createVuforia(parameters);
	}

	private static void initTfod(HardwareMap hwMap) {
		int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hwMap.appContext.getPackageName());

		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
	}

	/* RUN */

	private static String goldOrePosition;

	public static void run() {
		Sample.dropWall();

		//CODE
		switch(goldOrePosition)
		{
			case "CENTER":
				Drive.move(Drive.Direction.RIGHT, DistanceUnit.CM, 70);
				break;
			case "LEFT":

				break;
			case "RIGHT":

				break;

		}
		Sample.raiseWall();
	}

	public static void identify() {
		Sample.activate();

		List<Recognition> recognitions = null;

		int goldX = -1;
		int silver1X = -1;
		int silver2X = -1;

		CameraDevice.getInstance().setFlashTorchMode(true);

		while (silver1X == -1) {
			recognitions = tfod.getUpdatedRecognitions();

			if (recognitions != null) {
				if (recognitions.size() == 3) {
					for (Recognition recognition : recognitions) {
						if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
							goldX = (int) recognition.getRight();
						} else if (silver1X == -1) {
							silver1X = (int) recognition.getRight();
						} else {
							silver2X = (int) recognition.getRight();
						}
					}
				}
			}
		}

		CameraDevice.getInstance().setFlashTorchMode(false);

		Robot.telemetry.addData("G: ", Integer.toString(goldX));
		Robot.telemetry.addData("S1: ", Integer.toString(silver1X));
		Robot.telemetry.addData("S2: ", Integer.toString(silver2X));
		Robot.telemetry.update();

		if (goldX < silver1X && goldX < silver2X) {
			Robot.telemetry.addData("Gold Ore Position", "Left");
			Robot.telemetry.update();
			Sample.goldOrePosition = "LEFT";
		}
		else if (goldX > silver1X && goldX > silver2X) {
			Robot.telemetry.addData("Gold Ore Position", "Right");
			Robot.telemetry.update();
			Sample.goldOrePosition = "RIGHT";
		}
		else {
			Robot.telemetry.addData("Gold Ore Position", "Center");
			Robot.telemetry.update();
			Sample.goldOrePosition = "CENTER";
		}

		Sample.shutdown();
	}

	private static void activate() {
		tfod.activate();
	}

	private static void shutdown() {
		tfod.shutdown();
	}

	private static void dropWall() {
		Robot.servoWall.setPosition(0.5);
	}

	private static void raiseWall() {
		Robot.servoWall.setPosition(0);
	}
}
