package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robot.Brickbot;

import java.util.ArrayList;
import java.util.List;
//import java.util.Vector;
//import org.firstinspires.ftc.teamcode.robot.Drive;


@SuppressWarnings("All")
@Autonomous(name = "MyVuOp alskdjdlkaldkw", group = "CNU")

public class VuforiaNav extends LinearOpMode {

    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    private static final String VUFORIA_KEY = "AW+QDt3/////AAABmUOonI8rb0LTkFhL3nkIkY+ORnThf7+Fzmt4rOxdKkealCvQLhEIRrrk3YmbsfoMzTIzo1sHEcMOdubdxhR7QJxAivwT80q3G0w6zH5uYSDN1obUzkJbAcmgrD19wnSbpJIS1q152Vc1Ao/FdnSEqGBux9dM5Ng2N8ADBWtO0HzMk2iIx6z3MFGa5xV49nlxl0PhFUvcElWABhz4ToI4c3RTJyge0FcNiUfz/yqkjmM6dgqhC7gbtEoLyUncltUsWzLC4r51ebsBj+oSNE4K8nIzFiaWS+AZa3zz/ORjd8RVevLV261raejwbBQGwmkCHBIRuyZvdJdVU3y3YF6iaAx5j45V04amDLer3FeXzJGd";
    private OpenGLMatrix lastLocation = null;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private ElapsedTime period  = new ElapsedTime();
    private boolean targetVisible = false;

    public ModernRoboticsI2cGyro gyro= null;
    @Override
    public void runOpMode() throws InterruptedException {
        //AutonomFataCrater AFC = null;
        //Drive drive = null;
        Brickbot robot = null;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("RoverRuckus");

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        motorLeft = hardwareMap.dcMotor.get("motorfl");
        motorRight = hardwareMap.dcMotor.get("motorfr");

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*targets.get(0).setName("Back Perimeter Wall");
        targets.get(1).setName("Front Perimeter Wall");
        targets.get(2).setName("Red Perimeter Wall");
        targets.get(3).setName("Blue Perimeter Wall");*/

        VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        //final String VUFORIA_KEY = "AW+QDt3/////AAABmUOonI8rb0LTkFhL3nkIkY+ORnThf7+Fzmt4rOxdKkealCvQLhEIRrrk3YmbsfoMzTIzo1sHEcMOdubdxhR7QJxAivwT80q3G0w6zH5uYSDN1obUzkJbAcmgrD19wnSbpJIS1q152Vc1Ao/FdnSEqGBux9dM5Ng2N8ADBWtO0HzMk2iIx6z3MFGa5xV49nlxl0PhFUvcElWABhz4ToI4c3RTJyge0FcNiUfz/yqkjmM6dgqhC7gbtEoLyUncltUsWzLC4r51ebsBj+oSNE4K8nIzFiaWS+AZa3zz/ORjd8RVevLV261raejwbBQGwmkCHBIRuyZvdJdVU3y3YF6iaAx5j45V04amDLer3FeXzJGd";

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //VuforiaTrackableDefaultListener backPer = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
        //TODO: Do the others too
        //VuforiaTrackableDefaultListener frontPer = (VuforiaTrackableDefaultListener) targets.get(1).getListener();
        //VuforiaTrackableDefaultListener redPer = (VuforiaTrackableDefaultListener) targets.get(2).getListener();
        //VuforiaTrackableDefaultListener bluePer = (VuforiaTrackableDefaultListener) targets.get(3).getListener();

        /*setPower(1.0);

        targets.activate();

        setPower(0);

        while(opModeIsActive() && backPer.getRawPose() == null) {
            idle();
        }

        VectorF angles = null;
        angles = anglesFromTarget(backPer);
        VectorF trans = navOffWall(backPer.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        if(trans.get(0) > 0) {
            motorLeft.setPower(0.5);
            motorRight.setPower(-0.5);
        }
        else {
            motorLeft.setPower(-0.5);
            motorRight.setPower(0.5);
        }

        do {
            if(backPer.getPose() != null) {
                trans = navOffWall(backPer.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));
            }
            idle();
        }
        while(opModeIsActive() && Math.abs(trans.get(0)) > 30);

        setPower(0);

        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 409.575 * 560)));
        motorLeft.setTargetPosition((int) (motorLeft.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 409.575 * 560)));

        setPower(0.5);

        while(opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy()) {
            idle();
        }

        setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (backPer.getPose() == null || Math.abs(backPer.getPose().getTranslation().get(0)) > 10)) {
            if(backPer.getPose() != null) {
                if(backPer.getPose().getTranslation().get(0) > 0) {
                    motorLeft.setPower(-0.5);
                    motorRight.setPower(0.5);
                }
                else {
                    motorLeft.setPower(0.5);
                    motorRight.setPower(-0.5);
                }
            }
            else {
                motorLeft.setPower(-0.5);
                motorRight.setPower(0.5);
            }
        }

        setPower(0);*/

        //TODO: (VERIFY && CONTINUE THIS) || (VEFIRY && MODIFY)
        //TODO: (HINT) WATCH VUFORIA NAVIGATION ON YOUTUBE EP 4, 5, 6, ETC AND ADD
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        /*OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));*/

        //TODO: Verify the things before this moment and continue by adding to the active opmode
        while(opModeIsActive()) {


            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

        }



    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }

    private void setPower(@NonNull double... args) {
        if (args.length == 4) {
            motorLeft.setPower(args[0]);
            motorRight.setPower(args[1]);
        }
        else {
            throw new ArrayIndexOutOfBoundsException();
        }
    }

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long) period.milliseconds();

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


    private void stopMotors() {
        setPower(0, 0, 0, 0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Determines the error between the target angle and the robot's current heading.
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  Degrees in the range +/- 180. Centered on the robot's frame of reference.
     */
    private double getError(double targetAngle) {
        double error;
        error = targetAngle - gyro.getIntegratedZValue();
        return AngleUnit.normalizeDegrees(error);
    }

    /**
     * Returns desired steering force.  +/- 1 range.
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
