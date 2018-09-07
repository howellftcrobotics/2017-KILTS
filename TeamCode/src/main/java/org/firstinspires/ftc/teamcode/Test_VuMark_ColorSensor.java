package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@TeleOp(name="Test_VuMark_ColorSensor", group="Calibrators")
@Disabled
public class Test_VuMark_ColorSensor extends LinearOpMode
{
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;

    public Test_VuMark_ColorSensor()
    {
        m_coreConfig = new CoreConfig(
                CoreConfig.ROBOT_TEAM.KAOS,
                CoreConfig.ROBOT_OPERATING_MODE.TELEOP,
                CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1,
                CoreConfig.ROBOT_DRIVING_VIEW_MODE.STANDARD,
                telemetry, hardwareMap, this);
        m_robot = new BaseHardware(m_coreConfig);
    }

    public Test_VuMark_ColorSensor(CoreConfig coreConfig)
    {
        m_coreConfig = coreConfig;
        m_robot = new BaseHardware(m_coreConfig);
    }

    @Override
    public void runOpMode()
    {
        // initialize the base hardware
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
        m_robot.CenterRightJewelArmFlicker();
        m_robot.CenterLeftJewelArmFlicker();

        // fire up the vumark class and perform the first read for 1 second
        BaseVuMark vuMark = new BaseVuMark(m_coreConfig, true);
        RelicRecoveryVuMark glyphColumn  = vuMark.ReadVuMark(1, true);

        // Wait for the driver to press start
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                m_coreConfig.StartingPosition = CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1;
                m_robot.RaiseRightJewelArm();
            }
            if (gamepad1.b)
            {
                m_coreConfig.StartingPosition = CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1;
                m_robot.LowerRightJewelArm();
            }
            if (gamepad1.x)
            {
                m_coreConfig.StartingPosition = CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4;
                m_robot.RaiseLeftJewelArm();
            }
            if (gamepad1.y)
            {
                m_coreConfig.StartingPosition = CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4;
                m_robot.LowerLeftJewelArm();
            }

            if (gamepad1.left_bumper)
            {
                if (gamepad1.dpad_up)
                {
                    m_robot.CenterLeftJewelArmFlicker();
                }
                if (gamepad1.dpad_left)
                {
                    m_robot.PushLeftJewelArmFlicker();
                }
                if (gamepad1.dpad_right)
                {
                    m_robot.PullLeftJewelArmFlicker();
                }
            }
            else if (gamepad1.right_bumper)
            {
                if (gamepad1.dpad_up)
                {
                    m_robot.CenterRightJewelArmFlicker();
                }
                if (gamepad1.dpad_left)
                {
                    m_robot.PushRightJewelArmFlicker();
                }
                if (gamepad1.dpad_right)
                {
                    m_robot.PullRightJewelArmFlicker();
                }
            }
            else
            {
                if (gamepad1.dpad_up)
                {
                    m_robot.CenterJewelArmFlicker();
                }
                if (gamepad1.dpad_left)
                {
                    m_robot.PushJewelArmFlicker();
                }
                if (gamepad1.dpad_right)
                {
                    m_robot.PullJewelArmFlicker();
                }
            }


            // read the color sensors (based on the starting position above)
            BaseHardware.COLOR_RESULT color = m_robot.ReadJewelColor(0, false);
            telemetry.addData("Color", color.toString());

            // read the vumark
            glyphColumn = vuMark.ReadVuMark(0, false);
            telemetry.addData("Column", glyphColumn.toString());
            telemetry.update();
        }
        vuMark.Deactivate();
    }
}