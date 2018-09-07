package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="KILTS_TeleOp", group="KILTS - TeleOp")
@Disabled
public class KILTS_TeleOp extends LinearOpMode
{
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;

    public KILTS_TeleOp()
    {
        m_coreConfig = new CoreConfig(
                CoreConfig.ROBOT_TEAM.KILTS,
                CoreConfig.ROBOT_OPERATING_MODE.TELEOP,
                CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1,
                CoreConfig.ROBOT_DRIVING_VIEW_MODE.STANDARD,
                telemetry, hardwareMap, this);
        m_robot = new BaseHardware(m_coreConfig);
    }

    public KILTS_TeleOp(CoreConfig coreConfig)
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

        // initialize the devices
        m_robot.InitLiftGripper();
        m_robot.InitRelicGripper();
        m_robot.InitRelicArmKnuckle();
        m_robot.RaiseLeftJewelArm();
        m_robot.RaiseRightJewelArm();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            double leftDriveMotorPower = RobotUtil.ScaleJoystickInput(-gamepad1.left_stick_y);
            double rightDriveMotorPower = RobotUtil.ScaleJoystickInput(-gamepad1.right_stick_y);

            m_robot.leftFrontMotor.setPower(leftDriveMotorPower);
            m_robot.rightFrontMotor.setPower(rightDriveMotorPower);

            // open and close the lift grabber
            if (gamepad2.right_trigger > 0)
            {
                m_robot.CloseLiftGripper();
            }
            else
            {
                m_robot.OpenLiftGripper();
            }

            // open and close the relic grabber
            if (gamepad2.dpad_down)
            {
                m_robot.CloseRelicGripper();
            }
            else  if(gamepad2.dpad_up)
            {
                m_robot.OpenRelicGripper();
            }

            // raise/lower the lift
            double liftMotorPower = RobotUtil.ScaleJoystickInput(gamepad2.left_stick_y);
            m_robot.liftMotor.setPower(liftMotorPower);

            // extend/retract arm
            double relicArmMotorPower = RobotUtil.ScaleJoystickInput(gamepad2.right_stick_y);
            m_robot.relicArmMotor.setPower(relicArmMotorPower);

            // raise/lower relic knuckle
            if (gamepad2.x)
            {
                //m_robot.InitRelicArmKnuckle();
            }
            if (gamepad2.b)
            {
                m_robot.HoverRelicArmKnuckle();
            }
            if (gamepad2.y)
            {
                m_robot.RaiseRelicArmKnuckle();
            }
            if (gamepad2.a)
            {
                m_robot.LowerRelicArmKnuckle();
            }

            // ******* TESTING CODE - remove before match *******

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
            }
            else
            {
                telemetry.addData("Lift Touch Sensor:", "Is Not Pressed");
            }

            telemetry.update();
        }
    }
}