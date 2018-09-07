package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="KAOS-Auto-Blue-Right-Back-4", group="KAOS - Auto")
@Disabled
public class KAOS_Auto_Blue_Right_Back_4 extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // setup the properties of this OpMode
        CoreConfig coreConfig = new CoreConfig(
                CoreConfig.ROBOT_TEAM.KAOS,
                CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS,
                CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4,
                CoreConfig.ROBOT_DRIVING_VIEW_MODE.STANDARD,
                telemetry, hardwareMap, this);
        coreConfig.runOpMode();
    }
}
