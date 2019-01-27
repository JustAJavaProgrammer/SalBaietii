package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

    @Disabled
    @TeleOp(name = "Sensor: Digital touch", group = "Sensor")
    public class SensorDigitalTouchRO036  extends LinearOpMode {

        DigitalChannel digitalTouch;  // Hardware Device Object

        @Override
        public void runOpMode() {

            // get a reference to our digitalTouch object.
            digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

            // set the digital channel to input.
            digitalTouch.setMode(DigitalChannel.Mode.INPUT);

            // wait for the start button to be pressed.
            waitForStart();

            // while the op mode is active, loop and read the light levels.
            // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
            while (opModeIsActive()) {

                // send the info back to driver station using telemetry function.
                // if the digital channel returns true it's HIGH and the button is unpressed.
                if (digitalTouch.getState() == true) {
                    telemetry.addData("Digital Touch", "Is Not Pressed");
                } else {
                    telemetry.addData("Digital Touch", "Is Pressed");
                }

                telemetry.update();
            }
        }
    }

