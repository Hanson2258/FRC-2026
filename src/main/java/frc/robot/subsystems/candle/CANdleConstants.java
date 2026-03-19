package frc.robot.subsystems.candle;

import frc.robot.subsystems.candle.CANdleIO.Colour;

/** Constants for the CANdle subsystem */
public class CANdleConstants {
  /** Maximum brightness of all the LEDs for the brightness scalar */
  public static final double LED_FULL_BRIGHTNESS = 1; 
  /** Minimum brightness of all the LEDs for the brightness scalar */
  public static final double LED_OFF_BRIGHTNESS = 0;
  /** Default brightness of all the LEDs for the brightness scalar */
  public static final double LED_DEFAULT_BRIGHTNESS = 0.2;

  /** The color red */
  public static final Colour RED = new Colour(255, 0, 0);
  /** The color green */
  public static final Colour GREEN = new Colour(0, 255, 0);
  /** The color blue */
  public static final Colour BLUE = new Colour(0, 0, 255);
  /** The color purple */
  public static final Colour PURPLE = new Colour(128, 0, 128);
  /** The color white */
  public static final Colour WHITE = new Colour(255, 255, 255);
  /** Turns the LED brightness to 0, setting it off */
  public static final Colour OFF = new Colour(0, 0, 0);
}
