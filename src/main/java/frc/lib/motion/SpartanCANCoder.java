package frc.lib.motion;

// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CANCoderUsage;

/* Wrapper class around the CANCoder. */
public class SpartanCANCoder {

  private CANcoder cancoder;
  private CANcoderConfiguration config;

  boolean inverted;

  /**
   * Creates a new SpartanCANCoder.
   *
   * @param id CAN ID for the device
   * @param invert Whether to invert the encoder or not
   */
  public SpartanCANCoder(int id, boolean invert) {
    this.inverted = invert;
    cancoder = new CANcoder(id);
    config = new CANcoderConfiguration();
    buildConfig();
    configEncoder();
  }

  /**
   * Creates a new SpartanCANCoder;
   *
   * @param id CAN ID for the device
   * @param canivore Name of the Canivore
   * @param invert Whether to invert the encoder or not
   */
  public SpartanCANCoder(int id, String canivore, boolean invert) {
    this.inverted = invert;
    cancoder = new CANcoder(id, canivore);
    config = new CANcoderConfiguration();
    buildConfig();
    configEncoder();
  }

  private void configEncoder() {
    cancoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(cancoder, CANCoderUsage.kMinimal);
    cancoder.configAllSettings(config);
  }

  private void buildConfig() {
    config.absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0_to_360;
    config.sensorDirection = inverted;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
  }

  public StatusSignal<Double> getPosition() {
    //return cancoder.getPosition();
    return cancoder.getPosition();
  }

  public StatusSignal<Double> getVelocity() {
    return cancoder.getVelocity();
  }

  public StatusSignal<Double> getAbsolutePosition() {
    return cancoder.getAbsolutePosition();
  }
}
