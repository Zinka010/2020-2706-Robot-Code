/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Code by Charlie Macdonald.
 * Last modified on Feb 6th, 2020.
 */

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class IrSensor implements Sendable {
    private static final int kIRPort = 1; //set IR sensor port
        //TODO: The IR sensor's port may change, depending on the robot setup. (Use a robot-specific config to set this?)

    //median filter to discard outliers. Filters over 10 samples
    private final MedianFilter m_filter = new MedianFilter(10);

    private final AnalogInput m_irsensor;

    //Set the formula values to calculate IR sensor distance:
        //Values for short range (Sharp GP2Y0A41SK0F) 4-30cm sensor: 12.8528, -0.007545, -3.92, 30, 4.5.
        //Note: If a different IR sensor is used, these values will need to be updated.
    private final double irSlopeValue = 12.8528;
    private final double irXValue = 0.007545;
    private final double irConstant = -3.92;
    private final double irMaxDistanceCm = 30;
    private final double irMinDistanceCm = 4.5;
  
    public double getDistance() {
        //Calculate distance (cm) from mV:
        //Accurate to +/- 1.5cm when measuring within 5-20cm.
        double currentDistanceCm = ((1/
            (((m_filter.calculate(m_irsensor.getValue()))
            /1000 - irXValue)
            /irSlopeValue)
            ) //Inverse is effectively calculated at this point.
            +irConstant);

        //The following "if" statements fix the values outside the sensor's accurate measuring range.
        if (currentDistanceCm > irMaxDistanceCm) {
            currentDistanceCm = 30; //set distance to a fixed value if it is above the max measuring range.
        }
        if (currentDistanceCm < irMinDistanceCm) {
            currentDistanceCm = 0; //set distance to a fixed value if it is below the min measuring range.
            //Warning: Distances under 1cm will still produce erroneous values.
        }

        return(currentDistanceCm);

    }

    public IrSensor(int channel) {
        this.m_irsensor = new AnalogInput(kIRPort);
        SendableRegistry.addChild(this, m_irsensor);
    }

	@Override
	public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Distance", this::getDistance, null);
	}
}
