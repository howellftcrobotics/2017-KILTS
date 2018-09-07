package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="KILTS_TeleOp_POV", group="KILTS - TeleOp")
//@Disabled
public class KILTS_TeleOp_POV extends LinearOpMode
{
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;
    private boolean m_liftBlockDetected = false;

    public KILTS_TeleOp_POV()
    {
        m_coreConfig = new CoreConfig(
                CoreConfig.ROBOT_TEAM.KILTS,
                CoreConfig.ROBOT_OPERATING_MODE.TELEOP,
                CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1,
                CoreConfig.ROBOT_DRIVING_VIEW_MODE.POV,
                telemetry, hardwareMap, this);
        m_robot = new BaseHardware(m_coreConfig);
    }

    public KILTS_TeleOp_POV(CoreConfig coreConfig)
    {
        m_coreConfig = coreConfig;
        m_robot = new BaseHardware(m_coreConfig);
    }

    @Override
    public void runOpMode()
    {
        double drivePower;
        double turn;
        double left;
        double right;
        double max;

        if (!m_robot.init())
        {
            telemetry.addData("Error", "Hardware Init Failed");
            telemetry.update();
            sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        // initialize the gripper
        //m_robot.InitializeGryo();
        m_robot.InitLiftGripper();
        m_robot.InitRelicGripper();
        m_robot.InitRelicArmKnuckle();
        m_robot.RaiseLeftJewelArm();
        m_robot.RaiseRightJewelArm();
        m_robot.CloseVacuumValve();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the m_robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            //TODO: Original code from example, switch to specific scaling code for KILTS
            //drivePower = -gamepad1.left_stick_y;
            //turn = gamepad1.right_stick_x;

            // TEST code for scaling the power
            //drivePower = RobotUtil.ScaleJoystickInput_KILTS_Drive(-gamepad1.left_stick_y);
            //turn = RobotUtil.ScaleJoystickInput_KILTS_Drive(gamepad1.right_stick_x);

            // if left trigger is down, then scale back power
            if (gamepad1.left_trigger > 0)
            {
                // use 35% power
                drivePower = -gamepad1.left_stick_y * .35;
                turn = gamepad1.right_stick_x * .35;
            }
            else
            {
                // use 90% power
                drivePower = -gamepad1.left_stick_y * .90;
                turn = gamepad1.right_stick_x * .90;
            }

            if (gamepad1.left_bumper)
            {
                // turn on motor break mode
                m_robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m_robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m_robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m_robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if (gamepad1.right_bumper)
            {
                // turn off motor break mode and put to float mode
                m_robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m_robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m_robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m_robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Combine drive and turn for blended motion.
            left = drivePower + turn;
            right = drivePower - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // set th correct power to the motors
            m_robot.leftFrontMotor.setPower(left);
            m_robot.leftRearMotor.setPower(left);
            m_robot.rightFrontMotor.setPower(right);
            m_robot.rightRearMotor.setPower(right);

            if (!m_coreConfig.CompetitionMode)
            {
                telemetry.addData("LeftFrontPower", "%5.2f", right);
                telemetry.addData("RightFrontPower", "%5.2f",left);
                telemetry.addData("LeftRearPower", "%5.2f", right);
                telemetry.addData("RightRearPower", "%5.2f", left);

                telemetry.addData("LeftFrontPosition", "%7d", m_robot.leftFrontMotor.getCurrentPosition());
                telemetry.addData("RightFrontPosition", "%7d", m_robot.rightFrontMotor.getCurrentPosition());
                telemetry.addData("LeftRearPosition", "%7d", m_robot.leftRearMotor.getCurrentPosition());
                telemetry.addData("RightRearPosition", "%7d", m_robot.rightRearMotor.getCurrentPosition());

                telemetry.addData("RelicArmMotorPosition", "%7d", m_robot.relicArmMotor.getCurrentPosition());
                telemetry.addData("RelicArmMotorPower", "%5.2f", gamepad2.right_stick_y);
                telemetry.addData("LiftMotorPosition", "%7d", m_robot.liftMotor.getCurrentPosition());
                telemetry.addData("LiftMotorPower", "%5.2f", -gamepad2.left_stick_y);
            }

            if (gamepad2.left_trigger > 0)
            {
                m_robot.CloseLiftGripper();
            }
            else
            {
                m_robot.OpenLiftGripper();
            }


            if (gamepad2.right_trigger > 0)
            {
                m_robot.RunIntakeRollers();

                if (m_robot.liftVacuumMotor != null)
                {
                    m_robot.liftVacuumMotor.setPower(.75);
                }

                if (m_robot.liftVacuumServo != null)
                {
                    m_robot.CloseVacuumValve();
                }
            }
            else
            {
                m_robot.StopIntakeRollers();

                if (m_robot.liftVacuumMotor != null)
                {

                    m_robot.liftVacuumMotor.setPower(0);
                }

                if (m_robot.liftVacuumServo != null)
                {
                    m_robot.OpenVacuumValve();
                }
            }

            // open and close the relic grabber
            if (gamepad2.dpad_down)
            {
                m_robot.CloseRelicGripper();
            }
            else if (gamepad2.dpad_up)
            {
                m_robot.OpenRelicGripper();
            }

            // raise/lower relic knuckle
            if (gamepad2.x)
            {
                m_robot.InitRelicArmKnuckle();
            }
            if (gamepad2.b)
            {
                m_robot.HoverRelicArmKnuckle();
            }
            if (gamepad2.y)
            {
                m_robot.rightRearMotor.setPower(0);
                m_robot.rightFrontMotor.setPower(0);
                m_robot.leftRearMotor.setPower(0);
                m_robot.leftFrontMotor.setPower(0);
                m_robot.liftMotor.setPower(0);
                m_robot.liftVacuumMotor.setPower(0);

                m_robot.RaiseRelicArmKnuckle();
            }
            if (gamepad2.a)
            {
                m_robot.LowerRelicArmKnuckle();
            }

            // raise/lower the lift
            double liftMotorPower = -gamepad2.left_stick_y;
            m_robot.liftMotor.setPower(liftMotorPower);

            // extend/retract arm
            double relicArmMotorPower = gamepad2.right_stick_y;
            m_robot.relicArmMotor.setPower(relicArmMotorPower);

            // run the extend, drop and retract sequence
            if (gamepad2.right_bumper)
            {
                // stop all other motors since we can;t multi-thread this routine
                m_robot.rightRearMotor.setPower(0);
                m_robot.rightFrontMotor.setPower(0);
                m_robot.leftRearMotor.setPower(0);
                m_robot.leftFrontMotor.setPower(0);
                m_robot.liftMotor.setPower(0);
                m_robot.liftVacuumMotor.setPower(0);

                // this should work now as it it set to run without encoders so the sign of the
                // encoder count won't dictate the direction of the motor any more as it does in
                // RunMotorByEncoder (+/- on the motor power is ignored with run with encoders)
                m_robot.HoverRelicArmKnuckle();
                m_robot.RunRelicArmByTime(-.8, 1.3);
                m_robot.LoosenRelicGripper();
                m_coreConfig.OpMode.sleep(750);
                m_robot.LowerRelicArmKnuckle();
                m_coreConfig.OpMode.sleep(500);
                m_robot.OpenRelicGripper();
                m_coreConfig.OpMode.sleep(750);
                m_robot.RaiseRelicArmKnuckle(true);
                m_coreConfig.OpMode.sleep(250);
                m_robot.RunRelicArmByTime(.9, 1.3);

                /*
                Runnable autoRelicTask = new Runnable(){
                    @Override
                    public void run()
                    {
                        AutoPlaceRelic();
                    }
                };

                Thread th = new Thread(autoRelicTask);
                th.start();
                */
            }

            /*
            if (gamepad2.left_bumper)
            {
                // use this to dump the first relic and go for the second
                m_robot.HoverRelicArmKnuckle();
                m_robot.RunRelicArmByTime(-.8, 1.2);
                m_robot.LowerRelicArmKnuckle();
                m_coreConfig.OpMode.sleep(500);
                m_robot.OpenRelicGripper();
                m_coreConfig.OpMode.sleep(750);
                m_robot.RaiseRelicArmKnuckle(true);
                m_coreConfig.OpMode.sleep(250);

                // don't bring the arm in
                //m_robot.RunRelicArmByTime(.9, 1.3);

                // turn and go for second
                m_robot.DriveByEncoder(0.8, 8 , 8, 3);
                if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_FRONT_3) ||
                    (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4))
                {
                    m_robot.DriveByEncoder(0.7, 15 , -15, 3);
                }
                else
                {
                    m_robot.DriveByEncoder(0.7, -15 , 15, 3);
                }
                m_robot.DriveByEncoder(0.8, -8 , -8, 3);
                m_robot.HoverRelicArmKnuckle();
            }
            */

            // ******* TESTING CODE - remove before match ******
            if (!m_coreConfig.CompetitionMode)
            {
                if (gamepad1.a)
                {
                    m_robot.RaiseRightJewelArm();
                }
                if (gamepad1.b)
                {
                    m_robot.LowerRightJewelArm();
                }

                if (gamepad1.x)
                {
                    m_robot.RaiseLeftJewelArm();
                }
                if (gamepad1.y)
                {
                    m_robot.LowerLeftJewelArm();
                }

                if (gamepad1.dpad_right)
                {
                    m_coreConfig.StartingPosition = CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1;
                    BaseHardware.COLOR_RESULT color = m_robot.ReadJewelColor(0, false);
                    telemetry.addData("Color", color.toString());
                }

                if (gamepad1.dpad_left)
                {
                    m_coreConfig.StartingPosition = CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_FRONT_3;
                    BaseHardware.COLOR_RESULT color = m_robot.ReadJewelColor(0, false);
                    telemetry.addData("Color", color.toString());
                }

                if (m_robot.GetLiftTouchSensorState())
                {
                    telemetry.addData("Lift Touch Sensor:", "Is Pressed");
                } else {
                    telemetry.addData("Lift Touch Sensor:", "Is Not Pressed");
                }
            }
            telemetry.update();
        }
    }

    private void AutoPlaceRelic()
    {
        m_robot.HoverRelicArmKnuckle();
        m_robot.RunRelicArmByTime(-.8, 1.2);
        m_robot.LowerRelicArmKnuckle();
        m_coreConfig.OpMode.sleep(500);
        m_robot.OpenRelicGripper();
        m_coreConfig.OpMode.sleep(750);
        m_robot.RaiseRelicArmKnuckle(true);
        m_coreConfig.OpMode.sleep(250);
        m_robot.RunRelicArmByTime(.9, 1.3);
    }
}