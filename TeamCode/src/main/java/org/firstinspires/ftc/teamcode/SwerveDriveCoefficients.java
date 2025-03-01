package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SwerveDriveCoefficients {
    public final double WHEEL_CIRCUMFERENCE; // Wheel circumference in meters
    public final double TICKS_PER_REVOLUTION;
    public final Translation2d FRONT_LEFT_WHEEL_POSITION;
    public final Translation2d FRONT_RIGHT_WHEEL_POSITION;
    public final Translation2d BACK_LEFT_WHEEL_POSITION;
    public final Translation2d BACK_RIGHT_WHEEL_POSITION;
    public final String imuName;
    public PIDFCoefficients drivePIDFCoefficients, anglePIDCoefficients;

    /**
     * Constructor for SwerveDriveCoefficients.
     *
     * @param wheelCircumference      Wheel circumference in meters
     * @param ticksPerRevolution      Ticks per revolution of the wheel encoder
     * @param drivePIDFCoefficients   PIDF coefficients for the drive system
     * @param frontLeftWheelPosition  Position of the front-left wheel relative to the robot center
     * @param frontRightWheelPosition Position of the front-right wheel relative to the robot center
     * @param backLeftWheelPosition   Position of the back-left wheel relative to the robot center
     * @param backRightWheelPosition  Position of the back-right wheel relative to the robot center
     */
    public SwerveDriveCoefficients(double wheelCircumference, double ticksPerRevolution,
                                   PIDFCoefficients drivePIDFCoefficients,
                                   PIDFCoefficients anglePIDFCoefficients,
                                   Translation2d frontLeftWheelPosition,
                                   Translation2d frontRightWheelPosition,
                                   Translation2d backLeftWheelPosition,
                                   Translation2d backRightWheelPosition, String imuName) {
        WHEEL_CIRCUMFERENCE = wheelCircumference;
        TICKS_PER_REVOLUTION = ticksPerRevolution;

        this.drivePIDFCoefficients = drivePIDFCoefficients;
        this.anglePIDCoefficients = anglePIDFCoefficients;

        this.FRONT_LEFT_WHEEL_POSITION = frontLeftWheelPosition;
        this.FRONT_RIGHT_WHEEL_POSITION = frontRightWheelPosition;
        this.BACK_LEFT_WHEEL_POSITION = backLeftWheelPosition;
        this.BACK_RIGHT_WHEEL_POSITION = backRightWheelPosition;
        this.imuName = imuName;
    }
}
