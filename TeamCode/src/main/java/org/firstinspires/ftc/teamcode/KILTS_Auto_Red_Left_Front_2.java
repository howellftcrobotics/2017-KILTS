package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="KILTS-Auto-Red-Left-Front-2", group="KILTS - Auto")
//@Disabled
public class KILTS_Auto_Red_Left_Front_2 extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // setup the properties of this OpMode
        CoreConfig coreConfig = new CoreConfig(
                CoreConfig.ROBOT_TEAM.KILTS,
                CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS,
                CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2,
                CoreConfig.ROBOT_DRIVING_VIEW_MODE.STANDARD,
                telemetry, hardwareMap, this);
        coreConfig.runOpMode();
    }
}
