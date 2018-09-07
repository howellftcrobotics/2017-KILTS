package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Test_KILTS_Calibrator", group="Calibrators")
@Disabled
public class Test_KILTS_Calibrator extends LinearOpMode
{
    static final double COUNTS_PER_INCH = 50;
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;

    public Test_KILTS_Calibrator()
    {
        m_coreConfig = new CoreConfig(
                CoreConfig.ROBOT_TEAM.KILTS,
                CoreConfig.ROBOT_OPERATING_MODE.TELEOP,
                CoreConfig.ROBOT_STARTING_POSITION.UNKNOWN,
                CoreConfig.ROBOT_DRIVING_VIEW_MODE.POV,
                telemetry, hardwareMap, this);
        m_robot = new BaseHardware(m_coreConfig);
    }

    public Test_KILTS_Calibrator(CoreConfig coreConfig)
    {
        m_coreConfig = coreConfig;
        m_robot = new BaseHardware(m_coreConfig);
    }

    @Override
    public void runOpMode() {
        double drivePower;
        double turn;
        double left;
        double right;
        double max;

        int startPosition;
        double currentLeftPosition;
        double currentRightPosition;
        int gamePad1Mode = 1;

        if (!m_robot.init())
        {
            telemetry.addData("Error", "Hardware Init Failed");
            telemetry.update();
            sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        // reset the encoders so we know we are starting at zero
        m_robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to running with encoders as we need them to keep track of the counts
        m_robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // grab the current starting position, this should be zero
        startPosition = m_robot.leftFrontMotor.getCurrentPosition();

        // initialize the gyro
        m_robot.InitLiftGripper();
        m_robot.InitRelicGripper();
        m_robot.InitRelicArmKnuckle();
        m_robot.InitializeGryo();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // this routine will tell you:
        // -how far you have driven
        // -the position of the lift servos
        // -the position of the grabber servos
        while (opModeIsActive())
        {
            // determine if the gamepad 1 is going to control the drive train or jewel arm
            if ((gamepad1.guide) && (gamePad1Mode == 1))
            {
                gamePad1Mode = 2;
                sleep(1000);
            }
            if ((gamepad1.guide) && (gamePad1Mode == 2))
            {
                gamePad1Mode = 1;
                sleep(1000);
            }

            if (gamePad1Mode == 1)
            {
                // this test opMode is using POV to control the bot
                drivePower = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;
                left = drivePower + turn;
                right = drivePower - turn;
                max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0)
                {
                    left /= max;
                    right /= max;
                }
                m_robot.leftFrontMotor.setPower(left);
                m_robot.rightFrontMotor.setPower(right);

                // if the user presses the start button, reset the values
                if (gamepad1.start)
                {
                    // reset the starting position
                    startPosition = m_robot.leftFrontMotor.getCurrentPosition();

                    // recalibrate the gyro back to zero
                    m_robot.RecalibrateGryo();
                }

                // figure out how many inches have been driven since the reset
                m_robot.ComposeGyroTelemetry();
                currentLeftPosition = ((double) (m_robot.leftFrontMotor.getCurrentPosition() -  startPosition) / COUNTS_PER_INCH);
                currentRightPosition = ((double) (m_robot.rightFrontMotor.getCurrentPosition() - startPosition) / COUNTS_PER_INCH);
                telemetry.addData("Distance Driven:", "%7d:%7d", currentLeftPosition, currentRightPosition);  //TODO Format String Exception

                m_robot.GyroAngles = m_robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData(">", "Angle Turned: " + m_robot.FormatGyroAngle(m_robot.GyroAngles.angleUnit, m_robot.GyroAngles.firstAngle));
                telemetry.update();
            }
            else
            {
                // raise or lower the jewel arm servo so we can get their current position
                double leftJewelArmServoPosition = m_robot.leftJewelArmServo.getPosition();
                if (gamepad1.left_stick_y > 0)
                {
                    // go positive with the value
                    m_robot.leftJewelArmServo.setPosition(leftJewelArmServoPosition + 1);
                }
                else if (gamepad1.left_stick_y < 0)
                {
                    // go negative with the value
                    m_robot.leftJewelArmServo.setPosition(leftJewelArmServoPosition - 1);
                }
                double rightJewelArmServoPosition = m_robot.rightJewelArmServo.getPosition();
                if (gamepad1.right_stick_y > 0)
                {
                    // go positive with the value
                    m_robot.rightJewelArmServo.setPosition(rightJewelArmServoPosition + 1);
                }
                else if (gamepad1.right_stick_y < 0)
                {
                    // go negative with the value
                    m_robot.rightJewelArmServo.setPosition(rightJewelArmServoPosition - 1);
                }

                // change the direction for testing purposes
                if (gamepad1.x)
                {
                    m_robot.leftJewelArmServo.setDirection(Servo.Direction.REVERSE);
                }
                if (gamepad1.y)
                {
                    m_robot.leftJewelArmServo.setDirection(Servo.Direction.FORWARD);
                }
                if (gamepad1.a)
                {
                    m_robot.rightJewelArmServo.setDirection(Servo.Direction.REVERSE);
                }
                if (gamepad1.b)
                {
                    m_robot.rightJewelArmServo.setDirection(Servo.Direction.FORWARD);
                }

                // test the programmed jewel arm servos - raise and lower the arm
                if (gamepad1.right_trigger > 0)
                {
                    // raise the arm
                    m_robot.RaiseRightJewelArm();
                }
                else if (gamepad1.right_bumper)
                {
                    // lower the arm
                    m_robot.LowerRightJewelArm();
                }
                if (gamepad1.left_trigger > 0)
                {
                    // raise the arm
                    m_robot.RaiseLeftJewelArm();
                }
                else if (gamepad1.left_bumper)
                {
                    // lower the arm
                    m_robot.LowerLeftJewelArm();
                }

                // show the current servo positions so we can use them in programming
                telemetry.addData(">", "Left jewel arm servo: Position: %7d Direction: %s",
                        m_robot.leftJewelArmServo.getPosition(), m_robot.leftJewelArmServo.getDirection().toString());
                telemetry.addData(">", "Right jewel arm servo: Position: %7d Direction: %s",
                        m_robot.rightJewelArmServo.getPosition(), m_robot.rightJewelArmServo.getDirection().toString());
            }

            // move the gripper servos so we can get their current position
            double leftLiftGrabberServoPosition = m_robot.leftLiftGrabberServo.getPosition();
            if (gamepad2.left_stick_y > 0)
            {
                // go positive with the value
                m_robot.leftLiftGrabberServo.setPosition(leftLiftGrabberServoPosition + 1);
            }
            else if (gamepad2.left_stick_y < 0)
            {
                // go negative with the value
                m_robot.leftLiftGrabberServo.setPosition(leftLiftGrabberServoPosition - 1);
            }
            double rightLiftGrabberServoPosition = m_robot.rightLiftGrabberServo.getPosition();
            if (gamepad2.right_stick_y > 0)
            {
                // go positive with the value
                m_robot.rightLiftGrabberServo.setPosition(rightLiftGrabberServoPosition + 1);
            }
            else if (gamepad2.right_stick_y < 0)
            {
                // go negative with the value
                m_robot.rightLiftGrabberServo.setPosition(rightLiftGrabberServoPosition - 1);
            }

            // change the direction for testing purposes
            if (gamepad2.x)
            {
                m_robot.leftLiftGrabberServo.setDirection(Servo.Direction.REVERSE);
            }
            if (gamepad2.y)
            {
                m_robot.leftLiftGrabberServo.setDirection(Servo.Direction.FORWARD);
            }
            if (gamepad2.a)
            {
                m_robot.rightLiftGrabberServo.setDirection(Servo.Direction.REVERSE);
            }
            if (gamepad2.b)
            {
                m_robot.rightLiftGrabberServo.setDirection(Servo.Direction.FORWARD);
            }

            // test the programmed lift gripper servos - open and close the grabber
            if (gamepad2.right_trigger > 0)
            {
                // initialize the servos to the perfectly closed position
                m_robot.CloseLiftGripper();
            }
            else if (gamepad2.left_trigger > 0)
            {
                // open the servos to the straight out open position
                m_robot.OpenLiftGripper();
            }

            // show the current servo positions so we can use them in programming
            telemetry.addData(">", "Left lift gripper servo: Position: %7d Direction: %s",
                    m_robot.leftLiftGrabberServo.getPosition(), m_robot.leftLiftGrabberServo.getDirection().toString());
            telemetry.addData(">", "Right lift gripper servo: Position: %7d Direction: %s",
                    m_robot.rightLiftGrabberServo.getPosition(), m_robot.rightLiftGrabberServo.getDirection().toString());

        }
        telemetry.update();
    }
}