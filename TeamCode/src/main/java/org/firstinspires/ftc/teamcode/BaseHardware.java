package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class BaseHardware
{
    public enum COLOR_RESULT
    {
        UNKNOWN,
        BLUE,
        RED
    }

    // drive motors
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;

    // other dc motors
    public DcMotor liftMotor = null;
    public DcMotor relicArmMotor = null;
    public DcMotor liftVacuumMotor = null;

    // servo
    public Servo rightLiftGrabberServo;
    public Servo rightIntakeRollerServo;
    public Servo leftLiftGrabberServo;
    public Servo leftIntakeRollerServo;

    public Servo leftJewelArmServo;
    public Servo rightJewelArmServo;

    public Servo leftJewelArmServo2;
    public Servo rightJewelArmServo2;

    public Servo relicArmKnuckleServo;
    public Servo relicArmGrabberServo;

    public Servo liftVacuumServo;

    // color/distance sensor
    public ColorSensor leftJewelColorSensor;
    public ColorSensor rightJewelColorSensor;
    public DistanceSensor leftJewelDistanceSensor;
    public DistanceSensor rightJewelDistanceSensor;

    public DigitalChannel liftTouchSensor1;
    public DigitalChannel liftTouchSensor2;

    private final String IMU_GYRO = "imu";

    private final String LEFT_REAR_MOTOR = "left_rear";
    private final String LEFT_FRONT_MOTOR = "left_front";
    private final String RIGHT_REAR_MOTOR = "right_rear";
    private final String RIGHT_FRONT_MOTOR = "right_front";

    private final String LIFT_MOTOR = "lift_motor";
    private final String RELIC_MOTOR = "relic_arm_motor";

    private final String RIGHT_LIFT_GRABER_SERVO = "right_lift_grabber_servo";
    private final String RIGHT_LIFT_GRABER_SERVO2 = "right_lift_grabber_servo2";    //TODO: KILTS ONLY
    private final String LEFT_LIFT_GRABER_SERVO = "left_lift_grabber_servo";
    private final String LEFT_LIFT_GRABER_SERVO2 = "left_lift_grabber_servo2";      //TODO: KILTS ONLY

    private final String RIGHT_JEWEL_ARM_SERVO = "right_jewel_arm_servo";
    private final String LEFT_JEWEL_ARM_SERVO = "left_jewel_arm_servo";
    private final String RIGHT_JEWEL_ARM_FLICKER_SERVO = "right_jewel_arm_flicker_servo";   //TODO: NEW, KAOS ONLY
    private final String LEFT_JEWEL_ARM_FLICKER_SERVO = "left_jewel_arm_flicker_servo";     //TODO: NEW, KAOS ONLY

    private final String RELIC_ARM_KNUCKLE_SERVO = "relic_arm_knuckle_servo";
    private final String RELIC_ARM_GRABBER_SERVO = "relic_arm_grabber_servo";

    private final String RIGHT_JEWEL_ARM_COLOR_SENSOR = "right_jewel_arm_color";
    private final String LEFT_JEWEL_ARM_COLOR_SENSOR = "left_jewel_arm_color";

    private final String LIFT_TOUCH_SENSOR1 = "lift_touch_sensor1";  //TODO: NEW, NOT USED YET
    private final String LIFT_TOUCH_SENSOR2 = "lift_touch_sensor2";  //TODO: NEW, NOT USED YET

    private final String LIFT_VACUUM_MOTOR = "lift_vacuum_motor";
    private final String LIFT_VACUUM_SERVO = "lift_vacuum_servo";


    final double KAOS_LEFT_LIFT_SERVO_INIT = .50;
    final double KAOS_LEFT_LIFT_SERVO_OPEN = .02;  // was .15
    final double KAOS_LEFT_LIFT_SERVO_CLOSED = .30;  // was .30
    final double KAOS_LEFT_LIFT_SERVO_PACKAGED = .50;

    final double KAOS_LEFT_LIFT_SERVO_INIT2 = .35;
    final double KAOS_LEFT_LIFT_SERVO_OPEN2 = .88;  // was .75
    final double KAOS_LEFT_LIFT_SERVO_CLOSED2 = .60;  // was .60
    final double KAOS_LEFT_LIFT_SERVO_PACKAGED2 = .35;

    final double KAOS_RIGHT_LIFT_SERVO_INIT = 0;
    final double KAOS_RIGHT_LIFT_SERVO_OPEN = .50;  // was .35
    final double KAOS_RIGHT_LIFT_SERVO_CLOSED = .20; // was .20
    final double KAOS_RIGHT_LIFT_SERVO_PACKAGED = 0;

    final double KAOS_RIGHT_LIFT_SERVO_INIT2 = 1;
    final double KAOS_RIGHT_LIFT_SERVO_OPEN2 = .50; // was .65
    final double KAOS_RIGHT_LIFT_SERVO_CLOSED2 = .80;  // was .80
    final double KAOS_RIGHT_LIFT_SERVO_PACKAGED2 = 1;

    //TODO: THESE ARE THE VALUES THAT NEED TO BE ADJUSTED FOR SUPER REGIONALS FOR KILTS
    final double KILTS_LEFT_LIFT_SERVO_INIT = 0;
    final double KILTS_LEFT_LIFT_SERVO_CLOSED = .50;
    final double KILTS_LEFT_LIFT_SERVO_OPEN = 1;

    final double KILTS_RIGHT_LIFT_SERVO_INIT = .3;  //1
    final double KILTS_RIGHT_LIFT_SERVO_CLOSED = .7;
    final double KILTS_RIGHT_LIFT_SERVO_OPEN = .3;  //0
    //TODO: THESE ARE THE VALUES THAT NEED TO BE ADJUSTED FOR SUPER REGIONALS FOR KILTS

    final double KAOS_RIGHT_JEWEL_ARM_SERVO_UP = .60;
    final double KAOS_RIGHT_JEWEL_ARM_SERVO_DOWN = .01;
    final double KAOS_LEFT_JEWEL_ARM_SERVO_UP = .14;
    final double KAOS_LEFT_JEWEL_ARM_SERVO_DOWN = .83;

    final double KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_FORWARD = 1;    //TODO:  KAOS ONLY
    final double KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_CENTER = .45;   //TODO:  KAOS ONLY
    final double KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_BACKWARD = 0;   //TODO:  KAOS ONLY

    final double KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_FORWARD = 0;     //TODO:  KAOS ONLY
    final double KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_CENTER = .48;    //TODO:  KAOS ONLY
    final double KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_BACKWARD = 1;    //TODO:  KAOS ONLY

    final double KILTS_RIGHT_JEWEL_ARM_SERVO_UP = 0;
    final double KILTS_RIGHT_JEWEL_ARM_SERVO_DOWN = .80;

    final double KILTS_LEFT_JEWEL_ARM_SERVO_UP = 1;
    final double KILTS_LEFT_JEWEL_ARM_SERVO_DOWN = .25;

    final double KILTS_KNUCKLE_ARM_SERVO_INIT = 0;
    final double KILTS_KNUCKLE_ARM_SERVO_UP = .10;
    final double KILTS_KNUCKLE_ARM_SERVO_HOVER = .91;
    final double KILTS_KNUCKLE_ARM_SERVO_DOWN = 1;  //was .98

    final double KAOS_KNUCKLE_ARM_SERVO_INIT = 0;
    final double KAOS_KNUCKLE_ARM_SERVO_HOVER = .945;
    final double KAOS_KNUCKLE_ARM_SERVO_UP = .38;
    final double KAOS_KNUCKLE_ARM_SERVO_DOWN = .99;

    final double KILTS_RELIC_GRABBER_SERVO_INIT = 0;
    final double KILTS_RELIC_GRABBER_SERVO_OPEN = 0;
    final double KILTS_RELIC_GRABBER_SERVO_CLOSED = .90;
    final double KILTS_RELIC_GRABBER_SERVO_LOOSE = .50;

    final double KAOS_RELIC_GRABBER_SERVO_INIT = .95;
    final double KAOS_RELIC_GRABBER_SERVO_OPEN = .95;
    final double KAOS_RELIC_GRABBER_SERVO_CLOSED = .20;
    final double KAOS_RELIC_GRABBER_SERVO_LOOSE = .40;

    final double KILTS_VACUUM_SERVO_OPEN = .5;
    final double KILTS_VACUUM_SERVO_CLOSED = 1;
    final double KAOS_VACUUM_SERVO_OPEN = .5;
    final double KAOS_VACUUM_SERVO_CLOSED = 1;

    // references that need to be kept locally
    private CoreConfig m_coreConfig;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    //static final double DRIVE_SPEED = 0.6; // Nominal speed for better accuracy.
    //static final double GYRO_TURN_SPEED = 0.6; // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.09; // Larger is more responsive, but also less stable (default was .10)
    static final double P_DRIVE_COEFF = 0.15; // Larger is more responsive, but also less stable (default was .15)

    // P_TURN_COEFF should be 0.10 for KILTS (12-12-17)

    BNO055IMU Gyro = null;
    Orientation GyroAngles = null;
    Acceleration GyroGravity = null;
    BNO055IMU.Parameters GyroParameters = null;

    static final double KLITS_COUNTS_PER_INCH = 50;
    static final double KAOS_COUNTS_PER_INCH = 35;

    static final int COLOR_BLUE_HUE_LOWER = 140;
    static final int COLOR_BLUE_HUE_UPPER = 270;
    static final int COLOR_RED_HUE_LOWER = 300;
    static final int COLOR_RED_HUE_UPPER = 460;

    static final int KAOS_LIFT_MAX_COUNTS = 2150;
    static final int KAOS_LIFT_MIN_COUNTS = 40;
    static final int KAOS_RELIC_ARM_MAX_COUNTS = 4750;
    static final int KAOS_RELIC_ARM_MIN_COUNTS = 40;

    static final int KNUCKLE_MOVEMENT_CYCLE = 12;
    static final int KAOS_MOVEMENT_CYCLE = 18;

    public BaseHardware(CoreConfig coreConfig)
    {
        // we must know which m_robot we are working with
        m_coreConfig = coreConfig;
    }

    public boolean init()
    {
        boolean result = true;

        // do we know which m_robot we are?
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.UNKNOWN)
        {
            m_coreConfig.Telemetry.addData(">", "Team is not set, are we KAOS or KILTS?");
            return false;
        }

        // do we have a refernce to the opMode?
        if (m_coreConfig.OpMode == null)
        {
            m_coreConfig.Telemetry.addData(">", "Reference to OpMode is null.");
            return false;
        }

        // do we have a valid reference to the Hardware map?
        if (m_coreConfig.OpMode.hardwareMap == null)
        {
            m_coreConfig.Telemetry.addData(">", "Reference to OpMode.HardwareMap is null.");
            //return false;
        }
        
        // initialize the drive motors
        try
        {
            leftRearMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, LEFT_REAR_MOTOR);
            rightRearMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, RIGHT_REAR_MOTOR);
            leftFrontMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
            rightFrontMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);

            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // need to set one side of the m_robot to reverse since they are turned around
            if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
            {
                leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
                rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
                leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);

                //TODO: NOT CURRENTLY IN USE
                if (m_coreConfig.OperatingMode == CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS)
                {
                    // set the directions of the motors based on the fact that we will be switching
                    // drive train sets on every turn

                }
                else if (m_coreConfig.OperatingMode == CoreConfig.ROBOT_OPERATING_MODE.TELEOP)
                {
                    // set the motor direcions based on POV in TeleOp

                }
            }
            else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
            {
                // set the drive train up for basic tank drive control
                leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
                rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
                leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            // set all the drive motors to zero power
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);

            // set all motors to run without encoders for now - this is faster
            leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Initializing Drive Motors Exception: " + ex.getMessage() );
            result = false;
        }

        // initialize the imu gyro
        try
        {
            Gyro = m_coreConfig.OpMode.hardwareMap.get(BNO055IMU.class, IMU_GYRO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Initializing IMU Gyro Exception: " + ex.getMessage() );
            result = false;
        }

        // initialize the lift motor
        try
        {
            liftMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, LIFT_MOTOR);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
            {
                liftMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            else
            {
                liftMotor.setDirection(DcMotor.Direction.FORWARD);
            }
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Initializing Lift Motor Exception: " + ex.getMessage() );
            liftMotor = null;
        }

        // initialize the lift vacuum motor
        try
        {
            liftVacuumMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, LIFT_VACUUM_MOTOR);
            liftVacuumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftVacuumMotor.setDirection(DcMotor.Direction.FORWARD);
            liftVacuumMotor.setPower(0);
            liftVacuumMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Initializing Lift Vacuum Motor Exception: " + ex.getMessage() );
            liftVacuumMotor = null;
        }

        // configure the lift vacuum servo
        try
        {
            liftVacuumServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, LIFT_VACUUM_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Vacuum Servo Exception: " + ex.getMessage() );
            liftVacuumServo = null;
        }

        // initialize the lift touch sensor
        try
        {
            liftTouchSensor1 = m_coreConfig.OpMode.hardwareMap.get(DigitalChannel.class, LIFT_TOUCH_SENSOR1);
            liftTouchSensor1.setMode(DigitalChannel.Mode.INPUT);
            liftTouchSensor2 = m_coreConfig.OpMode.hardwareMap.get(DigitalChannel.class, LIFT_TOUCH_SENSOR2);
            liftTouchSensor2.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Initializing Lift Touch Sensor Exception: " + ex.getMessage() );
            liftTouchSensor1 = null;
            liftTouchSensor2 = null;
        }

        // configure the left lift grabber servo
        try
        {
            leftLiftGrabberServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, LEFT_LIFT_GRABER_SERVO);
//TODO: SERVO DIRECTION??
            if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
            {
                leftLiftGrabberServo.setDirection(Servo.Direction.REVERSE);  // might have to reverse these for KILTS, or not
            }
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Left Lift Servo Exception: " + ex.getMessage() );
            leftLiftGrabberServo = null;
        }

        // configure the left lift grabber servo 2
        try
        {
            leftIntakeRollerServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, LEFT_LIFT_GRABER_SERVO2);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Left Lift Servo 2 Exception: " + ex.getMessage() );
            leftIntakeRollerServo = null;
        }

        // configure the right lift grabber servo
        try
        {
            rightLiftGrabberServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, RIGHT_LIFT_GRABER_SERVO);
//TODO: SERVO DIRECTION??
            if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
            {
                rightLiftGrabberServo.setDirection(Servo.Direction.REVERSE);  // might have to reverse these for KILTS, or not
            }
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Right Lift Servo Exception: " + ex.getMessage() );
            rightLiftGrabberServo = null;
        }

        // configure the right lift grabber servo 2
        try
        {
            rightIntakeRollerServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, RIGHT_LIFT_GRABER_SERVO2);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Right Lift Servo 2 Exception: " + ex.getMessage() );
            rightIntakeRollerServo = null;
        }

        // initialize the relic arm motor
        try
        {
            relicArmMotor = m_coreConfig.OpMode.hardwareMap.get(DcMotor.class, RELIC_MOTOR);
            relicArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            relicArmMotor.setDirection(DcMotor.Direction.FORWARD);
            relicArmMotor.setPower(0);
            relicArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception ex) {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Initializing Relic Arm Motor Exception: " + ex.getMessage());
            relicArmMotor = null;
        }

        // configure the relic arm knuckle servo
        try
        {
            relicArmKnuckleServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, RELIC_ARM_KNUCKLE_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Relic Knuckle Servo Exception: " + ex.getMessage() );
            relicArmKnuckleServo = null;
        }

        // configure the relic arm grabber servo
        try
        {
            relicArmGrabberServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, RELIC_ARM_GRABBER_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Relic Arm Grabber Servo Exception: " + ex.getMessage() );
            relicArmGrabberServo = null;
        }

        // configure the right jewel arm color/distance sensor
        try
        {
            rightJewelColorSensor = m_coreConfig.OpMode.hardwareMap.get(ColorSensor.class, RIGHT_JEWEL_ARM_COLOR_SENSOR);
            rightJewelDistanceSensor = m_coreConfig.OpMode.hardwareMap.get(DistanceSensor.class, RIGHT_JEWEL_ARM_COLOR_SENSOR);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Right Jewel Arm Color/Distance Exception: " + ex.getMessage() );
            rightJewelColorSensor = null;
        }

        // configure the left jewel arm color/distance sensor
        try
        {
            leftJewelColorSensor = m_coreConfig.OpMode.hardwareMap.get(ColorSensor.class, LEFT_JEWEL_ARM_COLOR_SENSOR);
            leftJewelDistanceSensor = m_coreConfig.OpMode.hardwareMap.get(DistanceSensor.class, LEFT_JEWEL_ARM_COLOR_SENSOR);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Left Jewel Arm Color/Distance Exception: " + ex.getMessage() );
            leftJewelColorSensor = null;
        }

        // configure the left jewel arm servo
        try
        {
            leftJewelArmServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, LEFT_JEWEL_ARM_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Left Jewel Arm Servo Exception: " + ex.getMessage() );
            leftJewelArmServo = null;
        }

        // configure the left jewel arm servo pusher
        try
        {
            leftJewelArmServo2 = m_coreConfig.OpMode.hardwareMap.get(Servo.class, LEFT_JEWEL_ARM_FLICKER_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Left Jewel Arm Pusher Servo Exception: " + ex.getMessage() );
            leftJewelArmServo2 = null;
        }

        // configure the right jewel arm servo
        try
        {
            rightJewelArmServo = m_coreConfig.OpMode.hardwareMap.get(Servo.class, RIGHT_JEWEL_ARM_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Right Jewel Arm Servo Exception: " + ex.getMessage() );
            rightJewelArmServo = null;
        }

        // configure the right jewel arm pusher servo
        try
        {
            rightJewelArmServo2 = m_coreConfig.OpMode.hardwareMap.get(Servo.class, RIGHT_JEWEL_ARM_FLICKER_SERVO);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_coreConfig.Telemetry.addData(">", "Right Jewel Arm Pusher Servo Exception: " + ex.getMessage() );
            rightJewelArmServo2 = null;
        }

        // turn off LED lights on color sensors when not in autonomous
        if (m_coreConfig.OperatingMode == CoreConfig.ROBOT_OPERATING_MODE.AUTONOMOUS)
        {
            EnableColorSensorLEDs();
        }
        else
        {
            DisableColorSensorLEDs();
        }

        return  result;
    }

    public void EnableColorSensorLEDs()
    {
        if (leftJewelColorSensor != null)
        {
            leftJewelColorSensor.enableLed(true);
        }
        if (rightJewelColorSensor != null)
        {
            rightJewelColorSensor.enableLed(true);
        }
    }

    public void DisableColorSensorLEDs()
    {
        if (leftJewelColorSensor != null)
        {
            leftJewelColorSensor.enableLed(false);
        }
        if (rightJewelColorSensor != null)
        {
            rightJewelColorSensor.enableLed(false);
        }
    }

    public void InitializeGryo()
    {
        // initialize the gyro
        if (Gyro == null)
        {
            m_coreConfig.Telemetry.addData(">", "Gyro is null in InitializeGryo()");
        }
        else
        {
            m_coreConfig.Telemetry.addData(">", "Preparing gyro parameters...");
            GyroParameters = new BNO055IMU.Parameters();
            GyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            GyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            GyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            GyroParameters.loggingEnabled = true;
            GyroParameters.loggingTag = "IMU";
            GyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            m_coreConfig.Telemetry.addData(">", "Initializing gyro...");
            Gyro.initialize(GyroParameters);

            m_coreConfig.Telemetry.addData(">", "Starting acceleration integration on gyro...");
            Gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            m_coreConfig.Telemetry.addData(">", "Composing telemetry...");
            ComposeGyroTelemetry();
        }
    }

    public void RecalibrateGryo()
    {
        // initialize the gyro
        if (Gyro == null)
        {
            m_coreConfig.Telemetry.addData(">", "Gyro is null in RecalibrateGryo()");
        }
        else
        {
            // just make a call that uses the gyro, and it will self initialize
            Gyro.initialize(GyroParameters);

            while (!Gyro.isGyroCalibrated())
            {
                m_coreConfig.OpMode.sleep(50);
                m_coreConfig.OpMode.idle();
            }

            ComposeGyroTelemetry();
        }
    }

    public void InitRelicArmKnuckle()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(relicArmKnuckleServo, KAOS_KNUCKLE_ARM_SERVO_INIT);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(relicArmKnuckleServo, KILTS_KNUCKLE_ARM_SERVO_INIT);
        }
    }

    public void HoverRelicArmKnuckle()
    {
        double newPosition = relicArmKnuckleServo.getPosition();

        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            if (newPosition > KAOS_KNUCKLE_ARM_SERVO_HOVER)
            {
                while (newPosition > KAOS_KNUCKLE_ARM_SERVO_HOVER)
                {
                    newPosition -= .01;
                    RobotUtil.SetServoPosition(relicArmKnuckleServo, newPosition);
                    m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                    m_coreConfig.OpMode.idle();
                }
            }
            else
            {
                while (newPosition < KAOS_KNUCKLE_ARM_SERVO_HOVER)
                {
                    newPosition += .01;
                    RobotUtil.SetServoPosition(relicArmKnuckleServo, newPosition);
                    m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                    m_coreConfig.OpMode.idle();
                }
            }
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(relicArmKnuckleServo, KILTS_KNUCKLE_ARM_SERVO_HOVER);
        }
    }

    public void LowerRelicArmKnuckle()
    {
        double newPosition = relicArmKnuckleServo.getPosition();

        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            if (newPosition > KAOS_KNUCKLE_ARM_SERVO_DOWN)
            {
                while (newPosition > KAOS_KNUCKLE_ARM_SERVO_DOWN)
                {
                    newPosition -= .01;
                    RobotUtil.SetServoPosition(relicArmKnuckleServo, newPosition);
                    m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                    m_coreConfig.OpMode.idle();
                }
            }
            else
            {
                while (newPosition < KAOS_KNUCKLE_ARM_SERVO_DOWN)
                {
                    newPosition += .01;
                    RobotUtil.SetServoPosition(relicArmKnuckleServo, newPosition);
                    m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                    m_coreConfig.OpMode.idle();
                }
            }
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            while (newPosition < KILTS_KNUCKLE_ARM_SERVO_DOWN)
            {
                newPosition += .01;
                RobotUtil.SetServoPosition(relicArmKnuckleServo, newPosition);
                m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                m_coreConfig.OpMode.idle();
            }
        }
    }

    /*
       Runnable task = new Runnable(){
           @Override
           public void run()
           {
               m_coreConfig.Telemetry.addData(">", "Test Thread Task");
           }
       };

       Thread th = new Thread(task);
       th.start();
       */

    public void RaiseRelicArmKnuckleAsync()
    {
        Runnable task = new Runnable(){
            @Override
            public void run()
            {
                RaiseRelicArmKnuckle(false);
            }
        };

        Thread th = new Thread(task);
        th.start();
    }

    public void RaiseRelicArmKnuckle()
    {
        // call the old default
        RaiseRelicArmKnuckle(false);
    }

    public void RaiseRelicArmKnuckle(boolean alwaysFast)
    {
        // variable has to be final when passing it into a runnable thread
        double currentPosition = relicArmKnuckleServo.getPosition();

        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            if ((currentPosition < KAOS_KNUCKLE_ARM_SERVO_UP) || (alwaysFast))
            {
                RobotUtil.SetServoPosition(relicArmKnuckleServo, KAOS_KNUCKLE_ARM_SERVO_UP);
            }
            else
            {
                while (currentPosition > KAOS_KNUCKLE_ARM_SERVO_UP)
                {
                    currentPosition -= .01;
                    RobotUtil.SetServoPosition(relicArmKnuckleServo, currentPosition);
                    m_coreConfig.OpMode.sleep(KAOS_MOVEMENT_CYCLE);
                    m_coreConfig.OpMode.idle();
                }
            }
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            if ((currentPosition < KILTS_KNUCKLE_ARM_SERVO_UP) || (alwaysFast))
            {
                RobotUtil.SetServoPosition(relicArmKnuckleServo, KILTS_KNUCKLE_ARM_SERVO_UP);
            }
            else
            {
                double newPos = currentPosition;
                while (newPos > KILTS_KNUCKLE_ARM_SERVO_UP)
                {
                    newPos -= .01;
                    RobotUtil.SetServoPosition(relicArmKnuckleServo, newPos);
                    m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                    m_coreConfig.OpMode.idle();
                }
            }
        }
    }

    /*
    //TODO: TEST THIS
    // create a runnable object from the code that needs to be executed and pass off to a new thread
    Runnable task = new Runnable() {
        @Override
        public void run()
        {
            double newPos = newPosition;
            while (newPos > KAOS_KNUCKLE_ARM_SERVO_UP)
            {
                newPos -= .01;
                RobotUtil.SetServoPosition(relicArmKnuckleServo, newPos);
                m_coreConfig.OpMode.sleep(KNUCKLE_MOVEMENT_CYCLE);
                m_coreConfig.OpMode.idle();
            }
        }
    };

    // start the thread to run the
    Thread th = new Thread(task);
                th.start();
    */

    public boolean GetLiftTouchSensorState()
    {
        if ((liftTouchSensor1 != null) && (liftTouchSensor2 != null))
        {
            if ((!liftTouchSensor1.getState()) && (!liftTouchSensor2.getState()))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public void InitLiftGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KAOS_RIGHT_LIFT_SERVO_INIT);
            RobotUtil.SetServoPosition(rightIntakeRollerServo, KAOS_RIGHT_LIFT_SERVO_INIT2);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KAOS_LEFT_LIFT_SERVO_INIT);
            RobotUtil.SetServoPosition(leftIntakeRollerServo, KAOS_LEFT_LIFT_SERVO_INIT2);
        } 
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            // init needs to be the "Closed" position now
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KILTS_RIGHT_LIFT_SERVO_CLOSED);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KILTS_LEFT_LIFT_SERVO_CLOSED);
        }
    }

    public void OpenLiftGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KAOS_RIGHT_LIFT_SERVO_OPEN);
            RobotUtil.SetServoPosition(rightIntakeRollerServo, KAOS_RIGHT_LIFT_SERVO_OPEN2);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KAOS_LEFT_LIFT_SERVO_OPEN);
            RobotUtil.SetServoPosition(leftIntakeRollerServo, KAOS_LEFT_LIFT_SERVO_OPEN2);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KILTS_RIGHT_LIFT_SERVO_OPEN);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KILTS_LEFT_LIFT_SERVO_OPEN);
        }
    }

    public void CloseLiftGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KAOS_RIGHT_LIFT_SERVO_CLOSED);
            RobotUtil.SetServoPosition(rightIntakeRollerServo, KAOS_RIGHT_LIFT_SERVO_CLOSED2);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KAOS_LEFT_LIFT_SERVO_CLOSED);
            RobotUtil.SetServoPosition(leftIntakeRollerServo, KAOS_LEFT_LIFT_SERVO_CLOSED2);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KILTS_RIGHT_LIFT_SERVO_CLOSED);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KILTS_LEFT_LIFT_SERVO_CLOSED);
        }
    }

    public void PackageLiftGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KAOS_RIGHT_LIFT_SERVO_PACKAGED);
            RobotUtil.SetServoPosition(rightIntakeRollerServo, KAOS_RIGHT_LIFT_SERVO_PACKAGED2);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KAOS_LEFT_LIFT_SERVO_PACKAGED);
            RobotUtil.SetServoPosition(leftIntakeRollerServo, KAOS_LEFT_LIFT_SERVO_PACKAGED2);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightLiftGrabberServo, KILTS_RIGHT_LIFT_SERVO_CLOSED);
            RobotUtil.SetServoPosition(leftLiftGrabberServo, KILTS_LEFT_LIFT_SERVO_CLOSED);
        }
    }

    public void StopIntakeRollers()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightIntakeRollerServo, .5);
            RobotUtil.SetServoPosition(leftIntakeRollerServo, .5);
        }
    }

    public void RunIntakeRollers()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightIntakeRollerServo, 1);
            RobotUtil.SetServoPosition(leftIntakeRollerServo, -1);
        }
    }

    public void CloseRelicGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KAOS_RELIC_GRABBER_SERVO_CLOSED);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KILTS_RELIC_GRABBER_SERVO_CLOSED);
        }
    }

    public void InitRelicGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KAOS_RELIC_GRABBER_SERVO_INIT);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KILTS_RELIC_GRABBER_SERVO_INIT);
        }
    }

    public void OpenRelicGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KAOS_RELIC_GRABBER_SERVO_OPEN);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KILTS_RELIC_GRABBER_SERVO_OPEN);
        }
    }

    public void LoosenRelicGripper()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KAOS_RELIC_GRABBER_SERVO_LOOSE);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(relicArmGrabberServo, KILTS_RELIC_GRABBER_SERVO_LOOSE);
        }
    }

    public void RaiseJewelArm()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo, KAOS_RIGHT_JEWEL_ARM_SERVO_UP);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo, KAOS_LEFT_JEWEL_ARM_SERVO_UP);
            }
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo, KILTS_RIGHT_JEWEL_ARM_SERVO_UP);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo, KILTS_LEFT_JEWEL_ARM_SERVO_UP);
            }
        }
    }

    public void CenterJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (rightJewelArmServo2 != null) && (leftJewelArmServo2 != null))
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo2, KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_CENTER);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo2, KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_CENTER);
            }
        }
    }

    public void CenterLeftJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (leftJewelArmServo2 != null))
        {
            RobotUtil.SetServoPosition(leftJewelArmServo2, KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_CENTER);
        }
    }

    public void CenterRightJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (rightJewelArmServo2 != null))
        {
            RobotUtil.SetServoPosition(rightJewelArmServo2, KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_CENTER);
        }
    }

    public void PushJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (rightJewelArmServo2 != null) && (leftJewelArmServo2 != null))
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                    (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo2, KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_FORWARD);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo2, KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_FORWARD);
            }
        }
    }

    public void PushLeftJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (leftJewelArmServo2 != null))
        {
            RobotUtil.SetServoPosition(leftJewelArmServo2, KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_FORWARD);
        }
    }

    public void PushRightJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (rightJewelArmServo2 != null))
        {
            RobotUtil.SetServoPosition(rightJewelArmServo2, KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_FORWARD);
        }
    }

    public void PullJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (rightJewelArmServo2 != null) && (leftJewelArmServo2 != null))
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                    (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo2, KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_BACKWARD);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo2, KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_BACKWARD);
            }
        }
    }

    public void PullLeftJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)  && (leftJewelArmServo2 != null))
        {
            RobotUtil.SetServoPosition(leftJewelArmServo2, KAOS_LEFT_JEWEL_ARM_FLICKER_SERVO_BACKWARD);
        }
    }

    public void PullRightJewelArmFlicker()
    {
        if ((m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS) && (rightJewelArmServo2 != null))
        {
            RobotUtil.SetServoPosition(rightJewelArmServo2, KAOS_RIGHT_JEWEL_ARM_FLICKER_SERVO_BACKWARD);

        }
    }

    public void RaiseRightJewelArm()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(rightJewelArmServo, KAOS_RIGHT_JEWEL_ARM_SERVO_UP);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightJewelArmServo, KILTS_RIGHT_JEWEL_ARM_SERVO_UP);
        }
    }

    public void RaiseLeftJewelArm()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(leftJewelArmServo, KAOS_LEFT_JEWEL_ARM_SERVO_UP);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(leftJewelArmServo, KILTS_LEFT_JEWEL_ARM_SERVO_UP);
        }
    }

    public void LowerJewelArm()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo, KAOS_RIGHT_JEWEL_ARM_SERVO_DOWN);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo, KAOS_LEFT_JEWEL_ARM_SERVO_DOWN);
            }
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
                (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
            {
                RobotUtil.SetServoPosition(rightJewelArmServo, KILTS_RIGHT_JEWEL_ARM_SERVO_DOWN);
            }
            else
            {
                RobotUtil.SetServoPosition(leftJewelArmServo, KILTS_LEFT_JEWEL_ARM_SERVO_DOWN);
            }
        }
    }

    public void LowerRightJewelArm()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(rightJewelArmServo, KAOS_RIGHT_JEWEL_ARM_SERVO_DOWN);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(rightJewelArmServo, KILTS_RIGHT_JEWEL_ARM_SERVO_DOWN);
        }
    }

    public void LowerLeftJewelArm()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(leftJewelArmServo, KAOS_LEFT_JEWEL_ARM_SERVO_DOWN);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(leftJewelArmServo, KILTS_LEFT_JEWEL_ARM_SERVO_DOWN);
        }
    }

    public void OpenVacuumValve()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(liftVacuumServo, KAOS_VACUUM_SERVO_OPEN);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(liftVacuumServo, KILTS_VACUUM_SERVO_OPEN);
        }
    }

    public void CloseVacuumValve()
    {
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KAOS)
        {
            RobotUtil.SetServoPosition(liftVacuumServo, KAOS_VACUUM_SERVO_CLOSED);
        }
        else if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            RobotUtil.SetServoPosition(liftVacuumServo, KILTS_VACUUM_SERVO_CLOSED);
        }
    }

    public COLOR_RESULT ReadJewelColor(int timeoutSeconds, boolean updateTelemetry)
    {
        COLOR_RESULT result = COLOR_RESULT.UNKNOWN;

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = m_coreConfig.OpMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id",
                m_coreConfig.OpMode.hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) m_coreConfig.OpMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        // need to figure out which sensors we need to work with depending on which side of the field/robot we are on
        ColorSensor colorSensor;
        DistanceSensor distanceSensor;
        if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)  ||
            (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
        {
            colorSensor = rightJewelColorSensor;
            distanceSensor = rightJewelDistanceSensor;
        }
        else
        {
            colorSensor = leftJewelColorSensor;
            distanceSensor = leftJewelDistanceSensor;
        }

        // onky do the read once if no timeout set, otherwise loop for the specified amount of
        // time or until we make a determination
        if (timeoutSeconds < 1)
        {
            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);

            if (!m_coreConfig.CompetitionMode)
            {
                m_coreConfig.Telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
                m_coreConfig.Telemetry.addData("Alpha", colorSensor.alpha());
                m_coreConfig.Telemetry.addData("Red  ", colorSensor.red());
                m_coreConfig.Telemetry.addData("Green", colorSensor.green());
                m_coreConfig.Telemetry.addData("Blue ", colorSensor.blue());
                m_coreConfig.Telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });
            }

            // did we determine red or blue based on hue?
            if ((hsvValues[0] > COLOR_BLUE_HUE_LOWER) && (hsvValues[0] < COLOR_BLUE_HUE_UPPER))
            {
                result = COLOR_RESULT.BLUE;
            }
            else if ((hsvValues[0] > COLOR_RED_HUE_LOWER) && (hsvValues[0] < COLOR_RED_HUE_UPPER))
            {
                result = COLOR_RESULT.RED;
            }
            else
            {
                if ((colorSensor.red() > 30) && ((colorSensor.red() > colorSensor.blue())))
                {
                    result = COLOR_RESULT.RED;
                }
                else if ((colorSensor.blue() > 20) && ((colorSensor.blue() > colorSensor.red())))
                {
                    result = COLOR_RESULT.BLUE;
                }
                else
                {
                    result = COLOR_RESULT.UNKNOWN;
                }
            }
            if (!m_coreConfig.CompetitionMode)
            {
                m_coreConfig.Telemetry.addData("Color", result.toString());
            }
            if (updateTelemetry) m_coreConfig.Telemetry.update();
        }
        else
        {
            // loop for the specified amount of time or until we find a color
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while (m_coreConfig.OpMode.opModeIsActive()
                    && (elapsedTime.seconds() < timeoutSeconds)
                    && (result == COLOR_RESULT.UNKNOWN))
            {
                Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);

                if (!m_coreConfig.CompetitionMode)
                {
                    m_coreConfig.Telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
                    m_coreConfig.Telemetry.addData("Alpha", colorSensor.alpha());
                    m_coreConfig.Telemetry.addData("Red  ", colorSensor.red());
                    m_coreConfig.Telemetry.addData("Green", colorSensor.green());
                    m_coreConfig.Telemetry.addData("Blue ", colorSensor.blue());
                    m_coreConfig.Telemetry.addData("Hue", hsvValues[0]);

                    // change the background color to match the color detected by the RGB sensor.
                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                        }
                    });
                }

                // did we determine red or blue?
                if ((hsvValues[0] > COLOR_BLUE_HUE_LOWER) && (hsvValues[0] < COLOR_BLUE_HUE_UPPER))
                {
                    result = COLOR_RESULT.BLUE;
                }
                else if ((hsvValues[0] > COLOR_RED_HUE_LOWER) && (hsvValues[0] < COLOR_RED_HUE_UPPER))
                {
                    result = COLOR_RESULT.RED;
                }
                else
                {
                    if ((colorSensor.red() > 30) && ((colorSensor.red() > colorSensor.blue())))
                    {
                        result = COLOR_RESULT.RED;
                    }
                    else if ((colorSensor.blue() > 20) && ((colorSensor.blue() > colorSensor.red())))
                    {
                        result = COLOR_RESULT.BLUE;
                    }
                    else
                    {
                        result = COLOR_RESULT.UNKNOWN;
                    }
                }

                if (!m_coreConfig.CompetitionMode)
                {
                    m_coreConfig.Telemetry.addData("Color", result.toString());
                }
                if (updateTelemetry) m_coreConfig.Telemetry.update();
            }
        }

        if (!m_coreConfig.CompetitionMode)
        {
            // Set the panel back to the default color\
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }

        return  result;
    }

    public void RunLiftToTargetPosition(double speed, int encoderCounts, double timeout)
    {
        // run the lift as needed
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            // 1-28-18: the motor got reversed after the gearbox fried, need to inverse the motor
            // direction, so did it once here rather than touching all the code in the AutoCore class
            // 2/3/18 string broke and got reversed again
            ControlMotorByEncoder(liftMotor, speed, encoderCounts, timeout);
        }
        else
        {
            ControlMotorByEncoder(liftMotor, speed, encoderCounts, timeout);
        }
    }

    public void RunLiftByTime(double speed, double timeout)
    {
        // run the lift as needed
        ControlMotorByTime(liftMotor, speed, timeout);
    }

    public void RunRelicArmToTargetPosition(double speed, int encoderCounts, double timeout)
    {
        // run the lift as needed
        ControlMotorByEncoder(relicArmMotor, speed, encoderCounts, timeout);
    }

    public void RunRelicArmByTime(double speed, double timeout)
    {
        // run the lift as needed
        ControlMotorByTime(relicArmMotor, speed, timeout);
    }

    public void RunVacuumMotor(double speed)
    {
        liftVacuumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVacuumMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVacuumMotor.setPower(speed);
    }

    public void StopVacuumMotor()
    {
        liftVacuumMotor.setPower(0);
        liftVacuumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVacuumMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void ControlMotorByEncoder(DcMotor motor, double speed,
                                       int encoderCounts, double timeout)
    {
        int newTarget;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (m_coreConfig.OpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newTarget = motor.getCurrentPosition() + encoderCounts;
            motor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            motor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the m_robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the m_robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (m_coreConfig.OpMode.opModeIsActive() && (elapsedTime.seconds() < timeout) && (motor.isBusy()))
            {
                // Display it for the driver.
                if (!m_coreConfig.CompetitionMode)
                {
                    m_coreConfig.Telemetry.addData("Path1", "Running to %7d", newTarget);
                    m_coreConfig.Telemetry.addData("Path2", "Running at %7d", motor.getCurrentPosition());
                }
                m_coreConfig.OpMode.idle();
            }

            // Stop all motion;
            motor.setPower(0);

            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void ControlMotorByTime(DcMotor motor, double speed, double timeout)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure that the opmode is still active
        if (m_coreConfig.OpMode.opModeIsActive())
        {
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            motor.setPower(speed);

            // keep looping while we are still active, and there is time left
            while (m_coreConfig.OpMode.opModeIsActive() && (elapsedTime.seconds() < timeout))
            {
                // just wasting time
                m_coreConfig.OpMode.idle();
            }

            // Stop all motion;
            motor.setPower(0);
        }
    }

    public void ComposeGyroTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
       m_coreConfig.Telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the GyroAngles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            GyroAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            GyroGravity = Gyro.getGravity();
        }
        });

        /*
        Runnable task = new Runnable(){
            @Override
            public void run()
            {
                m_coreConfig.Telemetry.addData(">", "Test Thread Task");
            }
        };

        Thread th = new Thread(task);
        th.start();
        */

        if (!m_coreConfig.CompetitionMode)
        {
            m_coreConfig.Telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return Gyro.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return Gyro.getCalibrationStatus().toString();
                        }
                    });

            m_coreConfig.Telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.thirdAngle);
                        }
                    });

                /*
                m_coreConfig.Telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return GyroGravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(GyroGravity.xAccel * GyroGravity.xAccel
                                        + GyroGravity.yAccel * GyroGravity.yAccel
                                        + GyroGravity.zAccel * GyroGravity.zAccel));
                    }
                });
                */
        }
    }

    public String FormatGyroAngle(AngleUnit angleUnit, double angle)
    {
        return FormatGyroDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String FormatGyroDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /* getError determines the error between the target angle and the robot's current heading
        @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
        @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
        +ve error means the robot should turn LEFT (CCW) to reduce error.
    */
    public double getError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range
        if (Gyro == null)
        {
            m_coreConfig.Telemetry.addData(">", "Gyro is null in getError()");
        }
        robotError = targetAngle - Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /* returns desired steering force. +/- 1 range. +ve = steer left
        @param error Error angle in robot relative degrees *
        @param PCoeff Proportional Gain Coefficient * @return
    */
    public double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*  Method to spin on central axis to point in a new direction.
        Move will stop if either of these conditions occur:
        1) Move gets to the heading (angle)
        2) Driver stops the opmode running.
            @param speed Desired speed of turn.
            @param angle Absolute Angle (in Degrees) relative to last gyro reset.
            0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
            If a relative angle is required, add/subtract from current heading.
    */
    public void gyroTurn (double speed, double angle, double holdTime)
    {
        // keep looping while we are still active, and not on heading.
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (m_coreConfig.OpMode.opModeIsActive()
                && (elapsedTime.time() < holdTime)
                && !onHeading(speed, angle, P_TURN_COEFF))
        {
            // Update telemetry & Allow time for other processes to run.
            m_coreConfig.Telemetry.update();
            m_coreConfig.OpMode.idle();
        }
    }

    /* Method to obtain & hold a heading for a finite amount of time
        Move will stop once the requested time has elapsed
        @param speed Desired speed of turn.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
            0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
            If a relative angle is required, add/subtract from current heading.
        @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime)
    {
        // keep looping while we have time remaining.
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (m_coreConfig.OpMode.opModeIsActive()
                && (elapsedTime.time() < holdTime))
        {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            m_coreConfig.Telemetry.update();
            m_coreConfig.OpMode.idle();
        }

        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    /*  Perform one cycle of closed loop heading control.
        @param speed Desired speed of turn.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
        0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
        If a relative angle is required, add/subtract from current heading.
        @param PCoeff Proportional Gain coefficient
        @return
    */
    public boolean onHeading(double speed, double angle, double PCoeff)
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
        /*
        leftFrontMotor.setPower(leftSpeed);
        rightFrontMotor.setPower(rightSpeed);
        leftRearMotor.setPower(leftSpeed);
        rightRearMotor.setPower(rightSpeed);
        */

        //TODO: code borrowed from DriveByEncoder 12-1-2017
        if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
        {
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);
            leftRearMotor.setPower(leftSpeed);
            rightRearMotor.setPower(rightSpeed);
        }
        else
        {
            if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR)
            {
                leftFrontMotor.setPower(leftSpeed);
                rightRearMotor.setPower(rightSpeed);
            }
            else if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR)
            {
                rightFrontMotor.setPower(rightSpeed);
                leftRearMotor.setPower(leftSpeed);
            }
            else if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD)
            {
                leftFrontMotor.setPower(leftSpeed);
                rightFrontMotor.setPower(rightSpeed);
                leftRearMotor.setPower(leftSpeed);
                rightRearMotor.setPower(rightSpeed);
            }
            else //if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE)
            {
                leftFrontMotor.setPower(leftSpeed);
                rightFrontMotor.setPower(rightSpeed);
                leftRearMotor.setPower(leftSpeed);
                rightRearMotor.setPower(rightSpeed);
            }
        }

        // Display it for the driver.
        if (!m_coreConfig.CompetitionMode)
        {
            m_coreConfig.Telemetry.addData("Target", "%5.2f", angle);
            m_coreConfig.Telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            m_coreConfig.Telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        }
        return onTarget;
    }

    /*  Method to drive on a fixed compass bearing (angle), based on encoder counts.
        Move will stop if either of these conditions occur:
        1) Move gets to the desired position
        2) Driver stops the opmode running.
        @param speed Target speed for forward motion. Should allow for _/- variance for adjusting heading
        @param distance Distance (in inches) to move from current position. Negative distance means move backwards.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
                 If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive (double speed, double distance, double angle)
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

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure that the opmode is still active
        if (m_coreConfig.OpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * KLITS_COUNTS_PER_INCH);  //TODO: BAD using KILTS here like this
            newFrontLeftTarget = leftFrontMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = rightFrontMotor.getCurrentPosition() + moveCounts;
            newBackLeftTarget = leftRearMotor.getCurrentPosition() + moveCounts;
            newBackRightTarget = rightRearMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFrontMotor.setTargetPosition(newFrontLeftTarget);
            rightFrontMotor.setTargetPosition(newFrontRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftFrontMotor.setPower(speed);
            rightFrontMotor.setPower(speed);
            leftRearMotor.setPower(speed);
            rightRearMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (m_coreConfig.OpMode.opModeIsActive() &&
                (leftFrontMotor.isBusy() &&
                rightFrontMotor.isBusy()))
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
                leftFrontMotor.setPower(leftSpeed);
                rightFrontMotor.setPower(rightSpeed);
                leftRearMotor.setPower(leftSpeed);
                rightRearMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                if (!m_coreConfig.CompetitionMode)
                {
                    m_coreConfig.Telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    m_coreConfig.Telemetry.addData("Target", "%7d:%7d", newFrontLeftTarget, newFrontRightTarget);
                    m_coreConfig.Telemetry.addData("Actual", "%7d:%7d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
                    m_coreConfig.Telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                }
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void DriveByEncoder(double speed,
                               double leftInches, double rightInches,
                               double timeoutSeconds)
    {
        int newLeftTarget;
        int newRightTarget;
        DcMotor rightMotor;
        DcMotor leftMotor;

        //DcMotor rightMotor2;
        //DcMotor leftMotor2;

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure that the opmode is still active
        if (m_coreConfig.OpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS)
            {
                newLeftTarget = leftFrontMotor.getCurrentPosition() + (int) (leftInches * KLITS_COUNTS_PER_INCH);
                newRightTarget = rightFrontMotor.getCurrentPosition() + (int) (rightInches * KLITS_COUNTS_PER_INCH);
                leftFrontMotor.setTargetPosition(newLeftTarget);
                rightFrontMotor.setTargetPosition(newRightTarget);
                leftRearMotor.setTargetPosition(newLeftTarget);
                rightRearMotor.setTargetPosition(newRightTarget);
                rightMotor = rightFrontMotor;
                leftMotor = leftFrontMotor;

                //rightMotor2 = rightFrontMotor;
                //leftMotor2 = leftFrontMotor;
            }
            else
            {
                if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR)
                {
                    newLeftTarget = leftFrontMotor.getCurrentPosition() + (int) (leftInches * KAOS_COUNTS_PER_INCH);
                    newRightTarget = rightRearMotor.getCurrentPosition() + (int) (rightInches * KAOS_COUNTS_PER_INCH);
                    leftFrontMotor.setTargetPosition(newLeftTarget);
                    rightRearMotor.setTargetPosition(newRightTarget);
                    rightMotor = rightRearMotor;
                    leftMotor = leftFrontMotor;

                    //ightMotor2 = rightRearMotor;
                    //leftMotor2 = leftFrontMotor;

                } else if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR)
                {
                    newRightTarget = rightFrontMotor.getCurrentPosition() + (int) (rightInches * KAOS_COUNTS_PER_INCH);
                    newLeftTarget = leftRearMotor.getCurrentPosition() + (int) (leftInches * KAOS_COUNTS_PER_INCH);
                    rightFrontMotor.setTargetPosition(newRightTarget);
                    leftRearMotor.setTargetPosition(newLeftTarget);
                    rightMotor = rightFrontMotor;
                    leftMotor = leftRearMotor;

                    //rightMotor2 = rightFrontMotor;
                    //leftMotor2 = leftRearMotor;
                } else if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD)
                {
                    newLeftTarget = leftFrontMotor.getCurrentPosition() + (int) (leftInches * KAOS_COUNTS_PER_INCH);
                    newRightTarget = rightFrontMotor.getCurrentPosition() + (int) (rightInches * KAOS_COUNTS_PER_INCH);
                    leftFrontMotor.setTargetPosition(newLeftTarget);
                    rightFrontMotor.setTargetPosition(newRightTarget);
                    leftRearMotor.setTargetPosition(newLeftTarget);
                    rightRearMotor.setTargetPosition(newRightTarget);
                    rightMotor = rightFrontMotor;
                    leftMotor = leftFrontMotor;

                    //rightMotor2 = rightRearMotor;
                    //leftMotor2 = leftRearMotor;

                } else //if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE)
                {
                    // ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE (this is clear as mud, do not change)
                    // remember that when in drive to encoders, sign of the power is ignored and sign iof counts is used!!
                    newLeftTarget = rightFrontMotor.getCurrentPosition() + (int) (leftInches * KAOS_COUNTS_PER_INCH);
                    newRightTarget = rightRearMotor.getCurrentPosition() + (int) (rightInches * KAOS_COUNTS_PER_INCH);
                    leftFrontMotor.setTargetPosition(newLeftTarget);
                    rightFrontMotor.setTargetPosition(-newLeftTarget);
                    leftRearMotor.setTargetPosition(-newRightTarget);
                    rightRearMotor.setTargetPosition(newRightTarget);
                    rightMotor = rightRearMotor;
                    leftMotor = rightFrontMotor;

                    //rightMotor2 = leftRearMotor;
                    //leftMotor2 = leftFrontMotor;
                }
            }

            // reset the timeout time and start motion.
            if (m_coreConfig.Team == CoreConfig.ROBOT_TEAM.KILTS) {
                leftFrontMotor.setPower(Math.abs(speed));
                rightFrontMotor.setPower(Math.abs(speed));
                leftRearMotor.setPower(Math.abs(speed));
                rightRearMotor.setPower(Math.abs(speed));
            } else {
                if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR) {
                    leftFrontMotor.setPower(Math.abs(speed));
                    rightRearMotor.setPower(Math.abs(speed));
                } else if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR) {
                    rightFrontMotor.setPower(Math.abs(speed));
                    leftRearMotor.setPower(Math.abs(speed));
                } else if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD) {
                    leftFrontMotor.setPower(Math.abs(speed));   //R
                    rightFrontMotor.setPower(Math.abs(speed)); //F
                    leftRearMotor.setPower(Math.abs(speed));    //R
                    rightRearMotor.setPower(Math.abs(speed));  //F
                } else //if (m_coreConfig.DriveTrainSet == CoreConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE)
                {
                    // CoreConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE
                    leftFrontMotor.setPower(Math.abs(speed));    //R
                    rightFrontMotor.setPower(Math.abs(speed));   //F
                    leftRearMotor.setPower(Math.abs(speed));      //R
                    rightRearMotor.setPower(Math.abs(speed));     //F
                }
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the m_robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the m_robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while (m_coreConfig.OpMode.opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutSeconds) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                // Display it for the driver.
                if (!m_coreConfig.CompetitionMode) {
                    //m_coreConfig.Telemetry.addData(">", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    //m_coreConfig.Telemetry.addData(">", "Running at %7d :%7d",
                    //        leftMotor.getCurrentPosition(),
                    //        rightMotor.getCurrentPosition());

                    m_coreConfig.Telemetry.addData(">", "RunningF at %7d :%7d",
                            leftFrontMotor.getCurrentPosition(),
                            rightFrontMotor.getCurrentPosition());
                    m_coreConfig.Telemetry.addData(">", "RunningR at %7d :%7d",
                            leftRearMotor.getCurrentPosition(),
                            rightRearMotor.getCurrentPosition());
                    m_coreConfig.Telemetry.update();
                }
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

            // Turn off RUN_TO_POSITION, RUN_WITHOUT_ENCODER runs faster in TeleOp
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
