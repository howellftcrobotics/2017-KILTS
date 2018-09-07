package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class BaseVuMark
{
    private CoreConfig m_coreConfig = null;
    private VuforiaLocalizer m_vuforia = null;
    private VuforiaTrackables m_relicTrackables = null;
    private VuforiaTrackable m_relicTemplate = null;

    public BaseVuMark(CoreConfig coreConfig, boolean autoActivate)
    {
        // save for later use as the parent variable are out of scope
        m_coreConfig = coreConfig;

        try
        {
            // initialize the vuforia library
            int cameraMonitorViewId = m_coreConfig.OpMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", m_coreConfig.OpMode.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "ASalIQT/////AAAAGWbEakS1bEVCqpvKmEehu3YRaq5Q2YNdxKQk6klhOB2FwpjwxSQKWe7YWeXSB+VJxazFogY656zpwRTBTMMig4NqpvyYe1FhW8hoomhkbfjQQZfIgcc8tjM++gdc7PFpSwdoE1RgmJiW8Tj4s8HZlexmI9uD/ab9124I8QdpsKn4kgtz9rFH63gKhf+x+EuqzijH9PwgoWN+75QaHAlS/HED+IZXakLQPaLTKPglBW985m/qWBySiXjANbXtMMRSjSl+0PH9Mf+fYVh5Wq4q6GPwlIYGvfpnNVma/uDwMkoxB00r99xNt3U/WDfDPzKbxCzWdi7+HEHSnYkjn2rnYMwSIZDsx27sCVEgSWxZ9hA2";

            // use the camera on the back of the phone, this has a better resolution
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            m_vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            // load the 3 templates that were provided by FTC
            m_relicTrackables = m_vuforia.loadTrackablesFromAsset("RelicVuMark");
            m_relicTemplate = m_relicTrackables.get(0);
            m_relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            // need to activte?
            if (autoActivate) Activate();
        }
        catch  (Exception ex)
        {
            // failed to initialize the Vuforia library and hardware
            m_coreConfig.Telemetry.addData("BaseVuMark:", "Exception initializing the BaseVuMark Class: " + ex.getMessage());
        }
    }

    public void Activate()
    {
        // turn on the tracking as this use more power
        if (m_relicTrackables != null)
        {
            m_relicTrackables.activate();
        }
    }

    public void Deactivate()
    {
        // make sure to turn off the tracking camera to save power
        if (m_relicTrackables != null)
        {
            m_relicTrackables.deactivate();
        }
    }

    public RelicRecoveryVuMark ReadVuMark(int timeoutSeconds, boolean updateTelemetry)
    {
        // set the default to unknown so it is not null
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        ElapsedTime elapsedTime = new ElapsedTime();

        try
        {
            // check to see if what the camera sees matched any of the loaded templates
            if (timeoutSeconds < 1)
            {
                vuMark = RelicRecoveryVuMark.from(m_relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {
                    // found a match
                    m_coreConfig.Telemetry.addData("BaseVuMark:", "%s visible", vuMark);
                    //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) m_relicTemplate.getListener()).getPose();
                    //m_coreConfig.Telemetry.addData("BaseVuMark:", "Pose-" + format(pose));
                }
                else
                {
                    m_coreConfig.Telemetry.addData("BaseVuMark:", "not visible");
                }
            }
            else
            {
                // loop for the specified amount of time or until we find a color
                elapsedTime.reset();
                while (m_coreConfig.OpMode.opModeIsActive()
                    && (elapsedTime.seconds() < timeoutSeconds)
                    && (vuMark == RelicRecoveryVuMark.UNKNOWN))
                {
                    vuMark = RelicRecoveryVuMark.from(m_relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                    {
                        // found a match
                        m_coreConfig.Telemetry.addData("BaseVuMark:", "%s visible", vuMark);
                        //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) m_relicTemplate.getListener()).getPose();
                        //m_coreConfig.Telemetry.addData("BaseVuMark:", "Pose-" + format(pose));
                    }
                    else
                    {
                        m_coreConfig.Telemetry.addData("BaseVuMark:", "not visible");
                    }
                    if (updateTelemetry) m_coreConfig.Telemetry.update();
                }
            }

        }
        catch  (Exception ex)
        {
            // there was an error while trying to read the VuMarks, send the
            // exception information to the driver station
            m_coreConfig.Telemetry.addData("BaseVuMark:", "Exception Reading the BaseVuMark: " + ex.getMessage());
        }

        // return the default of unknown or the the matched value
        if (updateTelemetry) m_coreConfig.Telemetry.update();
        return vuMark;
    }

    private String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
