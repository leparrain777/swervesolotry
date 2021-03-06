/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int CANID_steerMotorLB = 4;
	public static final int CANID_driveMotorLF = 5;
	public static final int CANID_driveMotorLB = 6;
	public static final int CANID_steerMotorLF = 7;
	public static final int CANID_steerMotorRF = 8;
	public static final int CANID_driveMotorRF = 9;
	public static final int CANID_steerMotorRB = 10;
	public static final int CANID_driveMotorRB = 11;
	public static final int CANID_steerEncoderLF = 0;
	public static final int CANID_steerEncoderLB = 1;
	public static final int CANID_steerEncoderRF = 2;
	public static final int CANID_steerEncoderRB = 3;
	public static final double safetyoff = 1;
	public static final double safetyon = .5;
	public static final double deadbandxy = 0.05;
	public static final double deadbandz = 0.05;
	public static final int USB_driveController = 0;

	// public static final String MOTOR_LEFT_1_ID = null;
	// public static final int CANID_driveLF = 0;
	// public static final int CANID_driveRF = 0;
	// public static final int CANID_driveLB = 0;
	// public static final int CANID_driveRB = 0;
}
