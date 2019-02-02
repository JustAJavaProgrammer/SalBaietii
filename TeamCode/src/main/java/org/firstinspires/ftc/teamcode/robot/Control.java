package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;


@SuppressWarnings("all")
public class Control {
	private static Gamepad gamepad1;
	private static Gamepad gamepad2;

	public static void init(Gamepad gamepad1, Gamepad gamepad2) {
		Control.gamepad1 = gamepad1;
		Control.gamepad2 = gamepad2;
	}

	public static float     robotMove()         {return -gamepad1.left_stick_y;}
	public static float     robotSlide()        {return gamepad1.right_stick_x;}
	public static float     robotRotate()       {return gamepad1.left_stick_x;}
	public static boolean   liftUp()            {return gamepad1.right_bumper;}
	public static boolean   liftDown()          {return gamepad1.left_bumper;}

	public static boolean   armOpen()           {return gamepad2.right_stick_y < 0;}
	public static boolean   armClose()          {return gamepad2.right_stick_y > 0;}
	public static boolean   armExtend()         {return gamepad2.left_stick_y > 0;}
	public static boolean   armRetract()        {return gamepad2.left_stick_y < 0;}
	public static boolean   boxOpen()           {return gamepad2.left_bumper;}
	public static boolean   boxClose()          {return gamepad2.right_bumper;}
	public static boolean   collectorIn()       {return gamepad2.a;}
	public static boolean   collectorOut()      {return gamepad2.y;}
}
