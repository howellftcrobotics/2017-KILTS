package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CoreConfig
{
    //TODO: Make sure this is in the proper config
    public boolean CompetitionMode = false;
    //TODO: --------------------------------------

    public enum ROBOT_STARTING_POSITION
    {
        UNKNOWN,
        RED_LEFT_BACK_1,
        RED_LEFT_FRONT_2,
        BLUE_RIGHT_FRONT_3,
        BLUE_RIGHT_BACK_4
    }

    public enum ROBOT_TEAM
    {
        UNKNOWN,
        KILTS,
        KAOS
    }

    public enum ROBOT_OPERATING_MODE
    {
        UNKNOWN,
        TELEOP,
        AUTONOMOUS
    }

    public enum ROBOT_DRIVING_VIEW_MODE
    {
        UNKNOWN,
        STANDARD,
        POV
    }

    public enum ROBOT_DRIVE_TRAIN_SET
    {
        UNKNOWN,
        A_LF_RR,
        B_RF_LR,
        C_ALL_FORWARD_BACKWARD,
        D_ALL_SIDE_2_SIDE
    }

    public ROBOT_STARTING_POSITION StartingPosition = ROBOT_STARTING_POSITION.UNKNOWN;
    public ROBOT_TEAM Team = ROBOT_TEAM.UNKNOWN;
    public ROBOT_OPERATING_MODE OperatingMode = ROBOT_OPERATING_MODE.UNKNOWN;
    public ROBOT_DRIVE_TRAIN_SET DriveTrainSet = ROBOT_DRIVE_TRAIN_SET.UNKNOWN;
    public Telemetry Telemetry = null;
    public HardwareMap HardwareMap = null;
    public LinearOpMode OpMode = null;
    public ROBOT_DRIVING_VIEW_MODE DrivingMode = ROBOT_DRIVING_VIEW_MODE.UNKNOWN;

    public CoreConfig(ROBOT_TEAM robotTeam, ROBOT_OPERATING_MODE robotOperatingMode,
                      ROBOT_STARTING_POSITION robotStartingPosition,
                      ROBOT_DRIVING_VIEW_MODE drivingMode,
                      Telemetry telemetry,
                      HardwareMap hardwareMap, LinearOpMode baseOpMode)
    {
        StartingPosition = robotStartingPosition;
        Team = robotTeam;
        OperatingMode = robotOperatingMode;
        Telemetry = telemetry;
        HardwareMap = hardwareMap;
        OpMode = baseOpMode;
        DrivingMode = drivingMode;
    }

    public void runOpMode()
    {
        // determine which class we should forward application focus to
        if (this.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            if (this.OperatingMode == CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS)
            {
                KILTS_AutoCore kiltsRobot = new KILTS_AutoCore(this);
                kiltsRobot.runOpMode();
            }
            else
            {
                if (DrivingMode == ROBOT_DRIVING_VIEW_MODE.POV)
                {
                    KILTS_TeleOp_POV kiltsRobot = new KILTS_TeleOp_POV(this);
                    kiltsRobot.runOpMode();
                }
                else
                {
                    KILTS_TeleOp kiltsRobot = new KILTS_TeleOp(this);
                    kiltsRobot.runOpMode();
                }
            }
        }
        else if (this.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            if (this.OperatingMode == CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS)
            {
                KAOS_AutoCoreDirect kaosRobot = new KAOS_AutoCoreDirect(this);
                kaosRobot.runOpMode();
            }
            else
            {
                KAOS_TeleOp_POV kaosRobot = new KAOS_TeleOp_POV(this);
                kaosRobot.runOpMode();
            }
        }
    }
}
