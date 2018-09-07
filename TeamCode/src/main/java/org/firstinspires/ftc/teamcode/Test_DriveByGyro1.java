
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="Test_DriveByGyro1", group="Calibrators")
@Disabled
public class Test_DriveByGyro1 extends LinearOpMode
{
    CoreConfig coreConfig = new CoreConfig(
        CoreConfig.ROBOT_TEAM.KILTS,
        CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS,
        CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2,
        CoreConfig.ROBOT_DRIVING_VIEW_MODE.STANDARD,
        telemetry, hardwareMap, this);
    BaseHardware robot = new BaseHardware(coreConfig);
    
    // Use a Pushbot's hardware
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    static final int COUNTS_PER_INCH = 50;

    int startPosition;
    int newLeftTarget;
    int newRightTarget;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.5; // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.3; // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1; // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15; // Larger is more responsive, but also less stable

    @Override public void runOpMode()
    {
        robot.init();
        double drivePower;
        double turn;
        double left;
        double right;
        double max;

        // Set up the GyroParameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        telemetry.addData(">", "Configuring GyroParameters...");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "Gyro".
        telemetry.addData(">", "Getting the Gyro...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "Gyro");
        imu.initialize(parameters);

        telemetry.addData(">", "Waiting for gyro to calibrate...");
        composeTelemetry();
        telemetry.update();
        while (!imu.isGyroCalibrated())
        {
            sleep(50);
            telemetry.addData(">", "Still calibrating...");
            telemetry.update();
            idle();
        }
        telemetry.addData(">", "Calibration completed.");
        telemetry.update();

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted())
        {
            //telemetry.addData(">", "Robot Heading: " + formatAngle(GyroAngles.angleUnit, GyroAngles.firstAngle));
            telemetry.update();
            idle();
        }

        telemetry.addData(">", "Starting acceleration integration...");
        telemetry.update();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData(">", "Waiting for start...");
        telemetry.update();
        waitForStart();

        startPosition = robot.leftFrontMotor.getCurrentPosition();
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        telemetry.addData(">", "Starting driver operations...");
        telemetry.update();

        while (opModeIsActive())
        {

            drivePower = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drivePower + turn;
            right = drivePower - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // set th correct power to the motors
            robot.leftFrontMotor.setPower(left);
            robot.rightFrontMotor.setPower(right);

            if (gamepad1.x)
            {
                // reset the starting position
                startPosition = robot.leftFrontMotor.getCurrentPosition();
                imu.initialize(parameters);
                while (!imu.isGyroCalibrated())
                {
                    sleep(50);
                    telemetry.addData(">", "Still calibrating...");
                    telemetry.update();
                    idle();
                }
            }

            // figure out how many inches are driven
            newLeftTarget = (robot.leftFrontMotor.getCurrentPosition() - (int) startPosition) / COUNTS_PER_INCH;
            newRightTarget = (robot.rightFrontMotor.getCurrentPosition() - (int) startPosition) / COUNTS_PER_INCH;
            telemetry.addData(">", "Distance Driven (in): %7d :%7d", newLeftTarget, newRightTarget);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Angle Turned (in): " + formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.update();

            if (gamepad1.a)
            {
                gyroDrive(DRIVE_SPEED, 40.0, 0.0);
                gyroHold(TURN_SPEED, 0.0, 1.0);
            }
            else if (gamepad1.b)
            {
                gyroHold(TURN_SPEED, 45.0, 1.0);
            }
            else if (gamepad1.x)
            {
                gyroTurn(TURN_SPEED, 90.0);
                gyroHold(TURN_SPEED, 90.0, 1.0);
            }
            else if (gamepad1.y)
            {
                gyroTurn(TURN_SPEED, -180.0);
                gyroHold(TURN_SPEED, 180.0, 1.0);
            }
            telemetry.update();
        }
/*
        gyroDrive(DRIVE_SPEED, 40.0, 0.0); // Drive FWD X inches

        gyroTurn( GYRO_TURN_SPEED, 45.0); // Turn CCW to -45 Degrees

        gyroHold( GYRO_TURN_SPEED, -45.0, 2); // Hold -45 Deg heading for a 1/2 second

        gyroTurn( GYRO_TURN_SPEED, 45.0); // Turn CW to 45 Degrees

        gyroHold( GYRO_TURN_SPEED, 45.0, 0.5);// Hold 45 Deg heading for a 1/2 second

        gyroTurn( GYRO_TURN_SPEED, 0.0); // Turn CW to 0 Degrees

        gyroHold( GYRO_TURN_SPEED, 0.0, 1.0); // Hold 0 Deg heading for a 1 second

        gyroDrive(DRIVE_SPEED,-10.0, 0.0); // Drive REV 48 inches

        gyroHold( GYRO_TURN_SPEED, 0.0, 0.5);// Hold 0 Deg heading for a 1/2 second
*/
    }
    
    /** * Method to drive on a fixed compass bearing (angle), based on encoder counts. *
     * Move will stop if either of these conditions occur: *
     * 1) Move gets to the desired position *
     * 2) Driver stops the opmode running. * *
     * @param speed Target speed for forward motion. Should allow for _/- variance for adjusting heading *
     * @param distance Distance (in inches) to move from current position. Negative distance means move backwards. *
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset. *
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward. *
     *              If a relative angle is required, add/subtract from current heading. */
    public void gyroDrive ( double speed, double distance, double angle)
    {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.leftFrontMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.rightFrontMotor.getCurrentPosition() + moveCounts;
            //newBackLeftTarget = robot.leftRearMotor.getCurrentPosition() + moveCounts;
            //newBackRightTarget = robot.rightRearMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setTargetPosition(newFrontLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newFrontRightTarget);
            //robot.leftRearMotor.setTargetPosition(newBackLeftTarget);
            //robot.rightRearMotor.setTargetPosition(newBackRightTarget);
            //robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            //robot.leftRearMotor.setPower(speed);
            //robot.rightRearMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                (robot.leftFrontMotor.isBusy() &&
                robot.rightFrontMotor.isBusy()))
            {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0;
                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                robot.leftFrontMotor.setPower(leftSpeed);
                robot.rightFrontMotor.setPower(rightSpeed);
                //robot.leftRearMotor.setPower(leftSpeed);
                //robot.rightRearMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            //robot.leftRearMotor.setPower(0);
            //robot.rightRearMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /** * Method to spin on central axis to point in a new direction. * Move will stop if either of these conditions occur: * 1) Move gets to the heading (angle) * 2) Driver stops the opmode running. * * @param speed Desired speed of turn. * @param angle Absolute Angle (in Degrees) relative to last gyro reset. * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward. * If a relative angle is required, add/subtract from current heading. */
    public void gyroTurn ( double speed, double angle)
    {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF))
        {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    /** * Method to obtain & hold a heading for a finite amount of time *
     * Move will stop once the requested time has elapsed * *
     * @param speed Desired speed of turn. *
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset. *
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward. *
     *              If a relative angle is required, add/subtract from current heading. *
     *              @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime)
    {
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime))
        {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        //robot.leftRearMotor.setPower(0);
        //robot.rightRearMotor.setPower(0);
    }

    /** * Perform one cycle of closed loop heading control. * *
     * @param speed Desired speed of turn. *
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset. *
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward. *
     *              If a relative angle is required, add/subtract from current heading. *
     *              @param PCoeff Proportional Gain coefficient *
     *              @return */
    boolean onHeading(double speed, double angle, double PCoeff)
    {
        double error ;
        double steer ;
        boolean onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);
        //robot.leftRearMotor.setPower(leftSpeed);
        //robot.rightRearMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        return onTarget;
    }

    /** * getError determines the error between the target angle and the robot's current heading *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset). *
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference *
     * +ve error means the robot should turn LEFT (CCW) to reduce error. */
    public double getError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /** * returns desired steering force. +/- 1 range. +ve = steer left *
     * @param error Error angle in robot relative degrees *
     * @param PCoeff Proportional Gain Coefficient * @return */
    public double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the GyroAngles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}