// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.ConcurrentLinkedQueue;

//import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
//import org.apache.commons.math3.complex.Quaternion;
//import org.apache.commons.numbers.quaternion.Quaternion;
import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.apache.commons.geometry.euclidean.threed.rotation.*;
import java.lang.Math;



// CHECKSTYLE.OFF: TypeName
// CHECKSTYLE.OFF: MemberName
// CHECKSTYLE.OFF: SummaryJavadoc
// CHECKSTYLE.OFF: UnnecessaryParentheses
// CHECKSTYLE.OFF: OverloadMethodsDeclarationOrder
// CHECKSTYLE.OFF: NonEmptyAtclauseDescription
// CHECKSTYLE.OFF: MissingOverride
// CHECKSTYLE.OFF: AtclauseOrder
// CHECKSTYLE.OFF: LocalVariableName
// CHECKSTYLE.OFF: RedundantModifier
// CHECKSTYLE.OFF: AbbreviationAsWordInName
// CHECKSTYLE.OFF: ParameterName
// CHECKSTYLE.OFF: EmptyCatchBlock
// CHECKSTYLE.OFF: MissingJavadocMethod
// CHECKSTYLE.OFF: MissingSwitchDefault
// CHECKSTYLE.OFF: VariableDeclarationUsageDistance
// CHECKSTYLE.OFF: ArrayTypeStyle

/** This class is for the ADIS16470 IMU that connects to the RoboRIO SPI port. <br><br>
 * Modified by Team 340 (Greater Rochester Robotics) to include support for all 3 axes.<br><br>
 * <a href="https://github.com/Greater-Rochester-Robotics/SwerveBase2023/blob/main/src/main/java/frc/robot/subsystems/ADIS16470_IMU.java">GitHub Source</a>
 */
/*@SuppressWarnings({
  "unused",
  "PMD.RedundantFieldInitializer",
  "PMD.ImmutableField",
  "PMD.SingularField",
  "PMD.CollapsibleIfStatements",
  "PMD.MissingOverride",
  "PMD.EmptyIfStmt",
  "PMD.EmptyStatementNotInLoop"
})*/
public class ADIS16470_3Axis implements AutoCloseable, NTSendable { 
  /* ADIS16470 Register Map Declaration */
  private static final int FLASH_CNT = 0x00; // Flash memory write count
  private static final int DIAG_STAT = 0x02; // Diagnostic and operational status
  private static final int X_GYRO_LOW = 0x04; // X-axis gyroscope output, lower word
  private static final int X_GYRO_OUT = 0x06; // X-axis gyroscope output, upper word
  private static final int Y_GYRO_LOW = 0x08; // Y-axis gyroscope output, lower word
  private static final int Y_GYRO_OUT = 0x0A; // Y-axis gyroscope output, upper word
  private static final int Z_GYRO_LOW = 0x0C; // Z-axis gyroscope output, lower word
  private static final int Z_GYRO_OUT = 0x0E; // Z-axis gyroscope output, upper word
  private static final int X_ACCL_LOW = 0x10; // X-axis accelerometer output, lower word
  private static final int X_ACCL_OUT = 0x12; // X-axis accelerometer output, upper word
  private static final int Y_ACCL_LOW = 0x14; // Y-axis accelerometer output, lower word
  private static final int Y_ACCL_OUT = 0x16; // Y-axis accelerometer output, upper word
  private static final int Z_ACCL_LOW = 0x18; // Z-axis accelerometer output, lower word
  private static final int Z_ACCL_OUT = 0x1A; // Z-axis accelerometer output, upper word
  private static final int TEMP_OUT = 0x1C; // Temperature output (internal, not calibrated)
  private static final int TIME_STAMP = 0x1E; // PPS mode time stamp
    private static final int DATA_CNTR = 0x22; // Data Counter
  private static final int X_DELTANG_LOW = 0x24; // X-axis delta angle output, lower word
  private static final int X_DELTANG_OUT = 0x26; // X-axis delta angle output, upper word
  private static final int Y_DELTANG_LOW = 0x28; // Y-axis delta angle output, lower word
  private static final int Y_DELTANG_OUT = 0x2A; // Y-axis delta angle output, upper word
  private static final int Z_DELTANG_LOW = 0x2C; // Z-axis delta angle output, lower word
  private static final int Z_DELTANG_OUT = 0x2E; // Z-axis delta angle output, upper word
  private static final int X_DELTVEL_LOW = 0x30; // X-axis delta velocity output, lower word
  private static final int X_DELTVEL_OUT = 0x32; // X-axis delta velocity output, upper word
  private static final int Y_DELTVEL_LOW = 0x34; // Y-axis delta velocity output, lower word
  private static final int Y_DELTVEL_OUT = 0x36; // Y-axis delta velocity output, upper word
  private static final int Z_DELTVEL_LOW = 0x38; // Z-axis delta velocity output, lower word
  private static final int Z_DELTVEL_OUT = 0x3A; // Z-axis delta velocity output, upper word
  private static final int XG_BIAS_LOW =
      0x40; // X-axis gyroscope bias offset correction, lower word
  private static final int XG_BIAS_HIGH =
      0x42; // X-axis gyroscope bias offset correction, upper word
  private static final int YG_BIAS_LOW =
      0x44; // Y-axis gyroscope bias offset correction, lower word
  private static final int YG_BIAS_HIGH =
      0x46; // Y-axis gyroscope bias offset correction, upper word
  private static final int ZG_BIAS_LOW =
      0x48; // Z-axis gyroscope bias offset correction, lower word
  private static final int ZG_BIAS_HIGH =
      0x4A; // Z-axis gyroscope bias offset correction, upper word
  private static final int XA_BIAS_LOW =
      0x4C; // X-axis accelerometer bias offset correction, lower word
  private static final int XA_BIAS_HIGH =
      0x4E; // X-axis accelerometer bias offset correction, upper word
  private static final int YA_BIAS_LOW =
      0x50; // Y-axis accelerometer bias offset correction, lower word
  private static final int YA_BIAS_HIGH =
      0x52; // Y-axis accelerometer bias offset correction, upper word
  private static final int ZA_BIAS_LOW =
      0x54; // Z-axis accelerometer bias offset correction, lower word
  private static final int ZA_BIAS_HIGH =
      0x56; // Z-axis accelerometer bias offset correction, upper word
  private static final int FILT_CTRL = 0x5C; // Filter control
  private static final int MSC_CTRL = 0x60; // Miscellaneous control
  private static final int UP_SCALE = 0x62; // Clock scale factor, PPS mode
  private static final int DEC_RATE = 0x64; // Decimation rate control (output data rate)
  private static final int NULL_CNFG = 0x66; // Auto-null configuration control
  private static final int GLOB_CMD = 0x68; // Global commands
  private static final int FIRM_REV = 0x6C; // Firmware revision
  private static final int FIRM_DM = 0x6E; // Firmware revision date, month and day
  private static final int FIRM_Y = 0x70; // Firmware revision date, year
  private static final int PROD_ID = 0x72; // Product identification
  private static final int SERIAL_NUM = 0x74; // Serial number (relative to assembly lot)
  private static final int USER_SCR1 = 0x76; // User scratch register 1
  private static final int USER_SCR2 = 0x78; // User scratch register 2
  private static final int USER_SCR3 = 0x7A; // User scratch register 3
  private static final int FLSHCNT_LOW = 0x7C; // Flash update count, lower word
  private static final int FLSHCNT_HIGH = 0x7E; // Flash update count, upper word

  private class DataPacket {
    double timeScale, timeStamp , 
    X_av, Y_av, Z_av,
    X_a, Y_a, Z_a ;
 

    public String toString() {
      return String.format ( "A(%.2f,%.2f,%.2f) DA(%.2f,%.2f,%.2f) @ %.4f",
      X_a, Y_a, Z_a, X_av, Y_av, Z_av,
      timeStamp
      );
    }
  }
  //Diagnostics
  // private static final byte[] m_autospi_packet = {
  //   X_GYRO_OUT,
  //   FLASH_CNT,
  //   Y_GYRO_OUT,
  //   FLASH_CNT,  
  //   Z_GYRO_OUT,
  //   FLASH_CNT,
  //   X_ACCL_OUT,
  //   FLASH_CNT, 
  //   Y_ACCL_OUT,
  //   FLASH_CNT, 
  //   Z_ACCL_OUT,
  //   FLASH_CNT ,
  //   DATA_CNTR ,
  //   FLASH_CNT, 
  //   DIAG_STAT,
  //   FLASH_CNT
  // };

   //High-res outputs
  private static final byte[] m_autospi_packet = {
    X_GYRO_OUT, FLASH_CNT,  X_GYRO_LOW, FLASH_CNT,
    Y_GYRO_OUT,  FLASH_CNT,  Y_GYRO_LOW, FLASH_CNT,
    Z_GYRO_OUT, FLASH_CNT,  Z_GYRO_LOW, FLASH_CNT,
    X_ACCL_OUT, FLASH_CNT, X_ACCL_LOW, FLASH_CNT,
    Y_ACCL_OUT,  FLASH_CNT, 
    Z_ACCL_OUT, FLASH_CNT  
  };
 
  /**
   * the time the IMU calibrates on boot
   */
  public enum CalibrationTime {
    _32ms(0),
    _64ms(1),
    _128ms(2),
    _256ms(3),
    _512ms(4),
    _1s(5),
    _2s(6),
    _4s(7),
    _8s(8),
    _16s(9),
    _32s(10),
    _64s(11);

    private int value;

    private CalibrationTime(int value) {
      this.value = value;
    }
  }

  /**
   * contains the three real axes of the IMU 
   * and the aliases for Yaw, Pitch and Roll
   */
  public enum IMUAxis {
    kX,
    kY,
    kZ,
    kYaw,
    kPitch,
    kRoll
  }

  // Static Constants
  private static final double rad_to_deg = 180 / Math.PI ; 
  private static final double k_av_sf = 0.1 / Math.pow(2,16) / rad_to_deg;
  private static final double k_grav = 9.80665; // Convert g-unit to meters
  private static final double k_accel_sf = 1.25  / 1000 / Math.pow(2,16)   * k_grav; 

  // User-specified axes
  private IMUAxis m_yaw_axis;
  private IMUAxis m_pitch_axis;
  private IMUAxis m_roll_axis;

  // members used to accumulate/integrate gyro angles
  private double m_delta_ang_x = 0.0;
  private double m_delta_ang_y = 0.0;
  private double m_delta_ang_z = 0.0;

  private double [] m_ang_v_x = {0,0};
  private double [] m_ang_v_y = {0,0};
  private double [] m_ang_v_z = {0,0};

  // members used to store integrations 
  private double m_x = 0.0;
  private double m_y = 0.0;
  private double m_z = 0.0;

  private double m_angle_x_deg, m_angle_y_deg, m_angle_z_deg = 0;
 
  private double[] m_v_x = {0,0};
  private double[] m_v_y = {0,0};
  private double[] m_v_z = {0,0};

  private double[] m_accel_x = {0,0};
  private double[] m_accel_y = {0,0};
  private double[] m_accel_z = {0,0};

  // State variables
  private volatile boolean m_thread_active = false;
  private int m_calibration_time = 0;
  private volatile boolean m_first_run = true;
  private volatile boolean m_thread_idle = false;
  private boolean m_auto_configured = false;
  private double m_scaled_sample_rate = 2500.0;

  // Resources
  private SPI m_spi;
  private SPI.Port m_spi_port;
  private DigitalInput m_auto_interrupt;
  private DigitalOutput m_reset_out;
  private DigitalInput m_reset_in;
  private DigitalOutput m_status_led;
  private Thread m_acquire_task;
  private Thread m_math_task;
  private boolean m_connected;

  // SIM support variables.
  private SimDevice m_simDevice;
  private SimBoolean m_simConnected;
  private SimDouble m_simGyroAngleX;
  private SimDouble m_simGyroAngleY;
  private SimDouble m_simGyroAngleZ;
  private SimDouble m_simAccelX;
  private SimDouble m_simAccelY;
  private SimDouble m_simAccelZ;

  /**
   * Class that run in thread, calls acquisition of data
   */
  private static class AcquireTask implements Runnable {
    private ADIS16470_3Axis imu;

    public AcquireTask(ADIS16470_3Axis imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.acquire();
    }
  }


  private static class MathTask implements Runnable {
    private ADIS16470_3Axis imu;

    public MathTask(ADIS16470_3Axis imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.domath();
    }
  }

  /**
   * Creates a new ADIS16740 IMU object. 
   * 
   * The default setup is, Z-axis yaw, 
   * y-axis pitch, X-axis roll, onboard 
   * SPI port, and a calibration time 
   * of 4 seconds.
   */
  public ADIS16470_3Axis() {
    this(IMUAxis.kZ, IMUAxis.kY, IMUAxis.kX, SPI.Port.kOnboardCS0, CalibrationTime._4s);
  }

  /**
   * Creates a new ADIS16740 IMU object. 
   * 
   * The default setup is the onboard 
   * SPI port, and a calibration time 
   * of 4 seconds.
   * 
   * input axes limited to kX, kY and kZ
   * @param yaw_axis The axis that measures the yaw
   * @param pitch_axis The axis that measures the pitch
   * @param roll_axis The axis that measures the roll
   */
  public ADIS16470_3Axis(IMUAxis yaw_axis, IMUAxis pitch_axis, IMUAxis roll_axis){
    this(yaw_axis, pitch_axis, roll_axis, SPI.Port.kOnboardCS0, CalibrationTime._4s);
  }

  /**
   * Creates a new ADIS16740 IMU object. 
   * 
   * The default setup is the onboard 
   * SPI port.
   * 
   * input axes limited to kX, kY and kZ
   * @param yaw_axis The axis that measures the yaw
   * @param pitch_axis The axis that measures the pitch
   * @param roll_axis The axis that measures the roll
   * @param cal_time Calibration time
   */
  public ADIS16470_3Axis(IMUAxis yaw_axis, IMUAxis pitch_axis, IMUAxis roll_axis, CalibrationTime cal_time){
    this(yaw_axis, pitch_axis, roll_axis, SPI.Port.kOnboardCS0, cal_time);
  }

  /**
   * Creates a new ADIS16740 IMU object. 
   * 
   * input axes limited to kX, kY and kZ
   * @param yaw_axis The axis that measures the yaw
   * @param pitch_axis The axis that measures the pitch
   * @param roll_axis The axis that measures the roll
   * @param port The SPI Port the gyro is plugged into
   * @param cal_time Calibration time
   */
  public ADIS16470_3Axis(IMUAxis yaw_axis, IMUAxis pitch_axis, IMUAxis roll_axis, SPI.Port port, CalibrationTime cal_time) {
    if(yaw_axis == IMUAxis.kYaw || yaw_axis == IMUAxis.kPitch || yaw_axis == IMUAxis.kRoll ||
        pitch_axis == IMUAxis.kYaw || pitch_axis == IMUAxis.kPitch || pitch_axis == IMUAxis.kRoll ||
        roll_axis == IMUAxis.kYaw || roll_axis == IMUAxis.kPitch || roll_axis == IMUAxis.kRoll){
      DriverStation.reportError("ADIS16740 constructor only allows IMUAxis.kX, IMUAxis.kY or IMUAxis.kZ as arguments.", false);
      DriverStation.reportError("Constructing ADIS with default axes. (IMUAxis.kZ is defined as Yaw)", false);
      yaw_axis = IMUAxis.kZ;
      pitch_axis = IMUAxis.kY;
      roll_axis = IMUAxis.kX;
    }

    //save input axes to aliased locations.
    m_yaw_axis = yaw_axis;
    m_pitch_axis = pitch_axis;
    m_roll_axis = roll_axis;

    m_calibration_time = cal_time.value;
    m_spi_port = port;

    m_acquire_task = new Thread(new AcquireTask(this));
    m_math_task = new Thread(new MathTask(this));
    m_math_task.start(); 


    m_simDevice = SimDevice.create("Gyro:ADIS16470", port.value);
    if (m_simDevice != null) {
      m_simConnected = m_simDevice.createBoolean("connected", SimDevice.Direction.kInput, true);
      m_simGyroAngleX = m_simDevice.createDouble("gyro_angle_x", SimDevice.Direction.kInput, 0.0);
      m_simGyroAngleY = m_simDevice.createDouble("gyro_angle_y", SimDevice.Direction.kInput, 0.0);
      m_simGyroAngleZ = m_simDevice.createDouble("gyro_angle_z", SimDevice.Direction.kInput, 0.0);
      m_simAccelX = m_simDevice.createDouble("accel_x", SimDevice.Direction.kInput, 0.0);
      m_simAccelY = m_simDevice.createDouble("accel_y", SimDevice.Direction.kInput, 0.0);
      m_simAccelZ = m_simDevice.createDouble("accel_z", SimDevice.Direction.kInput, 0.0); 
    }

    if (m_simDevice == null) {
      // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
      // Relies on the RIO hardware by default configuring an output as low
      // and configuring an input as high Z. The 10k pull-up resistor internal to the
      // IMU then forces the reset line high for normal operation.
      m_reset_out = new DigitalOutput(27); // Drive SPI CS2 (IMU RST) low
          Timer.delay(0.01); // Wait 10ms
      m_reset_out.close(); 
      m_reset_in = new DigitalInput(27); // Set SPI CS2 (IMU RST) high
      Timer.delay(0.25); // Wait 250ms for reset to complete

      if (!switchToStandardSPI()) {
        return;
      }

      // Set IMU internal decimation to 4 (output data rate of 2000 SPS / (4 + 1) =
      // 400Hz)
      //writeRegister(DEC_RATE, 4);

      //get all 2000 samples per second, no filtering:
      m_scaled_sample_rate = (((0 + 1.0) / 2000.0) * 1000000.0);
      writeRegister(DEC_RATE, 0);

      // Set data ready polarity (HIGH = Good Data), Disable gSense Compensation and
      // PoP
      writeRegister(MSC_CTRL , 0x00C1);

      // Configure IMU internal Bartlett filter
      writeRegister(FILT_CTRL, 0);

      // Configure continuous bias calibration time based on user setting
      writeRegister(NULL_CNFG, (m_calibration_time | 0x0700));
 

      // Wait for samples to accumulate internal to the IMU (110% of user-defined
      // time)
      try {
        long dt = (long) (Math.pow(2.0, m_calibration_time) / 2000 * 64 * 1.1 * 1000);
        // Notify DS that IMU calibration delay is active
        DriverStation.reportWarning(
          String.format("ADIS16470 IMU Detected. Starting initial calibration delay of %d ms", dt), false);
        Thread.sleep(dt);
      } catch (InterruptedException e) {
      }

      DriverStation.reportWarning(
          "[...done]", false);
      // Write offset calibration command to IMU
      writeRegister(GLOB_CMD, 0x0001);

      // Configure and enable auto SPI
      if (!switchToAutoSPI()) {
        return;
    }

      // Let the user know the IMU was initiallized successfully
      DriverStation.reportWarning("ADIS16470 IMU Successfully Initialized!", false);

      // Drive "Ready" LED low
      m_status_led = new DigitalOutput(28); // Set SPI CS3 (IMU Ready LED) low
    }

    // Report usage and post data to DS
    HAL.report(tResourceType.kResourceType_ADIS16470, 0);
    m_connected = true;
  }

  public boolean isConnected() {
    if (m_simConnected != null) {
      return m_simConnected.get();
    }
    return m_connected;
  }

  /**
   * @param buf
   * @return
   */
  private static int toUShort(ByteBuffer buf) {
    return (buf.getShort(0)) & 0xFFFF;
  }

  /**
   * @param sint
   * @return
   */
  private static long toULong(int sint) {
    return sint & 0x00000000FFFFFFFFL;
  }

  /**
   * @param buf
   * @return
   */
  private static int toShort(int... buf) {
    return (short) (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }

  /**
   * @param buf
   * @return
   */
  private static int toInt(int... buf) {
    return (int)
        ((buf[0] & 0xFF) << 24 | (buf[1] & 0xFF) << 16 | (buf[2] & 0xFF) << 8 | (buf[3] & 0xFF));
  }

  /**
   * Switch to standard SPI mode.
   *
   * @return
   */
  private boolean switchToStandardSPI() {
    // Check to see whether the acquire thread is active. If so, wait for it to stop
    // producing data.
    if (m_thread_active) {
      m_thread_active = false;
      while (!m_thread_idle) {
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
        }
      }
      System.out.println("Paused the IMU processing thread successfully!");
      // Maybe we're in auto SPI mode? If so, kill auto SPI, and then SPI.
      if (m_spi != null && m_auto_configured) {
        m_spi.stopAuto();
        // We need to get rid of all the garbage left in the auto SPI buffer after
        // stopping it.
        // Sometimes data magically reappears, so we have to check the buffer size a
        // couple of times
        // to be sure we got it all. Yuck.
        int[] trashBuffer = new int[200];
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
        }
        int data_count = m_spi.readAutoReceivedData(trashBuffer, 0, 0);
        while (data_count > 0) {
          m_spi.readAutoReceivedData(trashBuffer, Math.min(data_count, 200), 0);
          data_count = m_spi.readAutoReceivedData(trashBuffer, 0, 0);
        }
        System.out.println("Paused auto SPI successfully.");
      }
    }
    // There doesn't seem to be a SPI port active. Let's try to set one up
    if (m_spi == null) {
      System.out.println("Setting up a new SPI port.");
      m_spi = new SPI(m_spi_port);
      m_spi.setClockRate(2000000);
      m_spi.setMode(SPI.Mode.kMode3);
      m_spi.setChipSelectActiveLow();
      for (int i=0; i<8; i++) { 
        Timer.delay(0.05);
        readRegister(PROD_ID); // Dummy read 
      } 

      // Validate the product ID
      if (readRegister(PROD_ID) != 16982) {
        DriverStation.reportError("Could not find ADIS16470 1: " + readRegister(PROD_ID) , false);
        close();
        return false;
      }
      return true;
    } else {
      // Maybe the SPI port is active, but not in auto SPI mode? Try to read the
      // product ID.
      readRegister(PROD_ID); // dummy read
      if (readRegister(PROD_ID) != 16982) {
        DriverStation.reportError("Could not find an ADIS16470 2 ", false);
        close();
        return false;
      } else {
        return true;
      }
    }
  }

  /** @return */
  boolean switchToAutoSPI() {
    // No SPI port has been set up. Go set one up first.
    if (m_spi == null) {
      if (!switchToStandardSPI()) {
        DriverStation.reportError("Failed to start/restart auto SPI", false);
        return false;
      }
    }
    // Only set up the interrupt if needed.
    if (m_auto_interrupt == null) {
      // Configure interrupt on SPI CS1
      m_auto_interrupt = new DigitalInput(26);
    }
    // The auto SPI controller gets angry if you try to set up two instances on one
    // bus.
    if (!m_auto_configured) {
      m_spi.initAuto(16000);
      m_auto_configured = true;
    }

    // Do we need to change auto SPI settings?
    m_spi.setAutoTransmitData(m_autospi_packet, 2);

    // Configure auto stall time
    m_spi.configureAutoStall(5, 1000, 1);
    // Kick off auto SPI (Note: Device configration impossible after auto SPI is
    // activated)
    // DR High = Data good (data capture should be triggered on the rising edge)
    m_spi.startAutoTrigger(m_auto_interrupt, true, false);

    // Check to see if the acquire thread is running. If not, kick one off.
    if (!m_acquire_task.isAlive()) {
      m_first_run = true;
      m_thread_active = true;
      m_acquire_task.start();
      System.out.println("Processing thread activated!");
    } else {
      // The thread was running, re-init run variables and start it up again.
      m_first_run = true;
      m_thread_active = true;
      System.out.println("Processing thread activated!");
    }
    // Looks like the thread didn't start for some reason. Abort.
    if (!m_acquire_task.isAlive()) {
      DriverStation.reportError("Failed to start/restart the acquire() thread.", false);
      close();
      return false;
    }
    return true;
  }

  /**
   * Configures calibration time
   *
   * @param new_cal_time New calibration time
   * @return 1 if the new calibration time is the same as the current one else 0
   */
  public int configCalTime(CalibrationTime new_cal_time) {
    if (m_calibration_time == new_cal_time.value) {
      return 1;
    }
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
      return 2;
    }
    m_calibration_time = new_cal_time.value;
    writeRegister(NULL_CNFG, (m_calibration_time | 0x700));
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
      return 2;
    }
    return 0;
  }

  public int configDecRate(int reg) {
    int m_reg = reg;
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
      return 2;
    }
    if (m_reg > 1999) {
      DriverStation.reportError("Attempted to write an invalid deimation value.", false);
      m_reg = 1999;
    }
    m_scaled_sample_rate = (((m_reg + 1.0) / 2000.0) * 1000000.0);
    writeRegister(DEC_RATE, m_reg);
    System.out.println("Decimation register: " + readRegister(DEC_RATE));
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
      return 2;
    }
    rot_initial=null; // force another gravity calibration
    Q.clear();
    return 0;
  }

  /**
   * Calibrate the gyro. It's important to make sure that the robot is not moving while the
   * calibration is in progress, this is typically done when the robot is first turned on while it's
   * sitting at rest before the match starts.
   */
  public void calibrate() {
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
    }
    writeRegister(GLOB_CMD, 0x0001);
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
    }
    ;
  }

  /**
   * @param reg
   * @return
   */
  private int readRegister(int reg) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    buf.order(ByteOrder.BIG_ENDIAN);
    buf.put(0, (byte) (reg & 0x7f));
    buf.put(1, (byte) 0);

    m_spi.write(buf, 2);
    m_spi.read(false, buf, 2);

    return toUShort(buf);
  }

  /**
   * @param reg
   * @param val
   */
  private void writeRegister(int reg, int val) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    // low byte
    buf.put(0, (byte) ((0x80 | reg)));
    buf.put(1, (byte) (val & 0xff));
    m_spi.write(buf, 2);
    // high byte
    buf.put(0, (byte) (0x81 | reg));
    buf.put(1, (byte) (val >> 8));
    m_spi.write(buf, 2);
  }

  /** 
   * Delete (free) the spi port used for the IMU. 
   */
  @Override
  public void close() {
    if (m_thread_active) {
      m_thread_active = false;
      try {
        if (m_acquire_task != null) {
          m_acquire_task.join();
          m_acquire_task = null; 
        }
      } catch (InterruptedException e) {
      }
      if (m_spi != null) {
        if (m_auto_configured) {
          m_spi.stopAuto();
        }
        m_spi.close();
        m_auto_configured = false;
        if (m_auto_interrupt != null) {
          m_auto_interrupt.close();
          m_auto_interrupt = null;
        }
        m_spi = null;
      }
    }
    System.out.println("Finished cleaning up after the IMU driver.");
  }

  private int acquireCnt = 0;
  private int overflowCnt = 0 ;
  private AtomicInteger queueSize = new AtomicInteger(0);
  private ConcurrentLinkedQueue<DataPacket> Q = new ConcurrentLinkedQueue<DataPacket>();
  private QuaternionRotation rot_initial= null, rot; 
  public int getAcquireCnt () { return acquireCnt;}
  public int getOverFlowCnt() { return overflowCnt; }
  public int getQuqueSize()  { return queueSize.get(); }

   
  private void domath() { 

    double calStartTime=-1;
    
    Vector3D v = Vector3D.of(0,0,0);
    int j=0;
    double gravity=-1;
    double lts=-1;
    int dpc = 0 ;

    while (true) {
      DataPacket dp = Q.poll();
      // Sleep loop for 10ms if nothing there
      if (dp == null )  {
        try {  Thread.sleep(10);}
        catch (InterruptedException e) { }
        continue;
      }
      // if (dpc++ % 1000 == 0)
      //   System.out.println (dp);
      
      queueSize.decrementAndGet(); 

      if (rot_initial == null){
        if (calStartTime <0) { 
            calStartTime = dp.timeStamp;
            v = Vector3D.of(0,0,0);
            j=0;
            System.out.println ("Starting gravity calibration"); 

          // members used to accumulate/integrate gyro angles
          m_delta_ang_x = 0.0;
          m_delta_ang_y = 0.0;
          m_delta_ang_z = 0.0;

          // members used to store integrations 
          m_accel_x = new double[] {0,0} ; 
          m_accel_y =  new double[] {0,0} ; 
          m_accel_z =  new double[] {0,0} ; 

          m_v_x = new double[] {0,0} ; 
          m_v_y= new double[] {0,0} ; 
          m_v_z=  new double[] {0,0} ;

          m_ang_v_x = new double[] {0,0} ; 
          m_ang_v_y = new double[] {0,0} ; 
          m_ang_v_z = new double[] {0,0} ; 

          m_angle_x_deg= m_angle_y_deg= m_angle_z_deg = 0;
          m_y = m_z = m_x  = 0; 
        }
        else if (dp.timeStamp - 1 < calStartTime) {
          //in gravity calibration
          v = v.add(Vector3D.of(dp.X_a, dp.Y_a, dp.Z_a));
          j++;
        } else {
          //calibration done 
          v = v.multiply(-1e0/j); 
          gravity = -v.norm();
          System.out.printf ("Gravity calibration done.  %d reads, g=%.3f at [%s]\n",
            j, gravity, v.normalize() );
           
          Vector3D straight_down = Vector3D.of(
              m_yaw_axis == IMUAxis.kX ? -1:0,
              m_yaw_axis == IMUAxis.kY ? -1:0,
              m_yaw_axis == IMUAxis.kZ ? -1:0 ) ; 

      
          rot=QuaternionRotation.createVectorRotation(v, straight_down);
          AxisAngleSequence rot_angles = rot.toAbsoluteAxisAngleSequence(AxisSequence.ZYX); 
          System.out.printf ("Rotation is: %s\n\t[%.2f, %.2f, %.2f]\n", rot, 
            rot_angles.getAngle3() * rad_to_deg, rot_angles.getAngle2()* rad_to_deg, rot_angles.getAngle1()* rad_to_deg);
          calStartTime =-1;
          rot_initial=rot;
          lts = dp.timeStamp;
        } 
      }  else {
        //post-calibration work section
        double dt = dp.timeStamp - lts;
        lts = dp.timeStamp; 
        
        m_ang_v_x[1] = m_ang_v_x[0]; m_ang_v_x [0] = dp.X_av;
        m_ang_v_y[1] = m_ang_v_y[0]; m_ang_v_y [0] = dp.Y_av;
        m_ang_v_z[1] = m_ang_v_z[0]; m_ang_v_z [0] = dp.Z_av;

        m_delta_ang_x = (m_ang_v_x[1] + m_ang_v_x[0]) / 2 * dt;
        m_delta_ang_y = (m_ang_v_y[1] + m_ang_v_y[0]) / 2 * dt;
        m_delta_ang_z = (m_ang_v_z[1] + m_ang_v_z[0]) / 2 * dt;

        // System.out.printf ("ANGLE: %.4f, %.4f, %.4f\n", 
        //   m_integ_angle_x * rad_to_deg,
        //   m_integ_angle_y* rad_to_deg,
        //   m_integ_angle_z * rad_to_deg
        //   );

        AxisAngleSequence da = AxisAngleSequence.createAbsolute(AxisSequence.ZYX, m_delta_ang_z, m_delta_ang_y, m_delta_ang_x);
        QuaternionRotation delta_rot = QuaternionRotation.fromAxisAngleSequence(da);
        rot = rot.multiply(delta_rot);

        Vector3D accel = Vector3D.of(dp.X_a, dp.Y_a, dp.Z_a); 
        accel = rot.apply(accel);
        accel = accel.add (Vector3D.of(0, 0, gravity)); 
 
        AxisAngleSequence rot_angles = (rot.multiply(rot_initial.inverse())).toAbsoluteAxisAngleSequence(AxisSequence.ZYX);

        synchronized (this) { 
            m_accel_x[1] = m_accel_x[0]; m_accel_x[0] = accel.getX();
            m_accel_y[1] = m_accel_y[0]; m_accel_y[0] = accel.getY();
            m_accel_z[1] = m_accel_z[0]; m_accel_z[0] = accel.getZ();

            m_v_x[1] = m_v_x[0]; m_v_x[0] += ( m_accel_x[1] +  m_accel_x[0] )  / 2 * dt;
            m_v_y[1] = m_v_y[0]; m_v_y[0] += ( m_accel_y[1] +  m_accel_y[0] )  / 2 * dt;
            m_v_z[1] = m_v_z[0]; m_v_z[0] += ( m_accel_z[1] +  m_accel_z[0] )  / 2 * dt;

            m_x += ( m_v_x[1] +  m_v_x[0] )  / 2 * dt;
            m_y += ( m_v_y[1] +  m_v_y[0] )  / 2 * dt;
            m_z += ( m_v_z[1] +  m_v_z[0] )  / 2 * dt; 

            m_angle_x_deg = rot_angles.getAngle3() * rad_to_deg; 
            m_angle_y_deg = rot_angles.getAngle2() * rad_to_deg; 
            m_angle_z_deg = rot_angles.getAngle1() * rad_to_deg; 
        }
 
      } 
        
    }
 
  }

  public double getX() { return m_x;}
  public double getY() { return m_y;}
  public double getZ() { return m_z;}
  public double getVX() { return m_v_x[0];}
  public double getVY() { return m_v_y[0];}
  public double getVZ() { return m_v_z[0];}



  private void acquire() {
    // Set data packet length
    final int dataset_len = 23; // n data points + timestamp + 2 
    final int BUFFER_SIZE = 4000;
    
    // Set up buffers and variables
    int[] buffer = new int[BUFFER_SIZE];
    int data_count = 0;
    int data_remainder = 0;
    int data_to_read = 0;
    double previous_timestamp = 0.0;
    //int last_data_cnt = 0;
    
    while (true) {
      // Sleep loop for 10ms
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
      }

      if (m_thread_active) {
        m_thread_idle = false;
      try {
          data_count =
              m_spi.readAutoReceivedData(
                  buffer, 0, 0); // Read number of bytes currently stored in the buffer
          
          data_remainder =
              data_count % dataset_len; // Check if frame is incomplete. Add 1 because of timestamp
          data_to_read = data_count - data_remainder; // Remove incomplete data from read count
          /* Want to cap the data to read in a single read at the buffer size */
          if (data_to_read > BUFFER_SIZE) {
            DriverStation.reportWarning(
                "ADIS16470 data processing thread overrun has occurred!", false);
            data_to_read = BUFFER_SIZE - (BUFFER_SIZE % dataset_len);
            overflowCnt += 1; 
          }
    
          m_spi.readAutoReceivedData(
              buffer, data_to_read, 0); // Read data from DMA buffer (only complete sets)

         
          // Could be multiple data sets in the buffer. Handle each one.
          for (int i = 0; i < data_to_read; i += dataset_len) {
            int p=i;
            // Timestamp is at buffer[i]
            double timeStamp = buffer[p] / 1e06; 
            double timeScale = (m_scaled_sample_rate / (buffer[p++] - previous_timestamp)); 
            p++ ; p++; //some kind of zero byte always here?
          
            DataPacket dp = new DataPacket();

            dp.timeScale = timeScale;
            dp.timeStamp = timeStamp;
          
            dp.X_av = toInt(buffer[p++], buffer[p++], buffer[p++], buffer[p++])  * k_av_sf;
            dp.Y_av = toInt(buffer[p++], buffer[p++], buffer[p++], buffer[p++] )  * k_av_sf;
            dp.Z_av = toInt(buffer[p++], buffer[p++], buffer[p++], buffer[p++] )  * k_av_sf;

            dp.X_a= toInt(buffer[p++], buffer[p++], buffer[p++], buffer[p++])  * k_accel_sf;
            dp.Y_a= toInt(buffer[p++], buffer[p++], 0, 0)  * k_accel_sf;
            dp.Z_a= toInt(buffer[p++], buffer[p++], 0, 0)  * k_accel_sf;

            Q.add(dp);
            queueSize.incrementAndGet();

            // int data_cntr = toShort (buffer[p++], buffer[p++]) ;
            // int diag_stat = toShort (buffer[p++], buffer[p++]) ;
            // if (diag_stat != 0 ) {
            //   DriverStation.reportWarning(
            //     String.format("Error returned from gyro DIAG_STAT: %d", diag_stat), false);
            // }

            // if (data_cntr -1 < last_data_cnt) {
            //   overflowCnt += data_cntr - 1 - last_data_cnt;
            // }
            // last_data_cnt = data_cntr; 


            acquireCnt += 1; 

            // Store timestamp for next iteration
            previous_timestamp = buffer[i];

          }
        
        }
        catch (edu.wpi.first.hal.util.UncleanStatusException e){

          DriverStation.reportWarning(
              "ADIS16470 data processing thread exception:\n" + e , false);
        }
         
        
      } else {
        m_thread_idle = true;
      
      }
    }
  }

  /** 
   * Resets all gyro axis accumulators to 0.0
   */
  public void resetAllAngles() {
    synchronized (this) {
      m_delta_ang_x = 0.0;
      m_delta_ang_y = 0.0;
      m_delta_ang_z = 0.0;
    }
  }

  /**
   * Allow the designated gyro angle to be set to a 
   * given value. This may happen with unread values 
   * in the buffer, it is suggested that the IMU is 
   * not moving when this method is run.
   * 
   * @param axis IMUAxis that will be changed
   * @param angle a double in degrees (CCW positive)
   */
  public void setGyroAngle(IMUAxis axis, double angle){
        //if pitch, yaw or roll is inputed then it is replaced with its equivelant axis
        switch (axis) {
          case kYaw: 
            axis = m_yaw_axis;
            break;
          case kPitch: 
            axis = m_pitch_axis;
            break;
          case kRoll: 
            axis = m_roll_axis;
            break;
          default: //is it isn't pitch yaw or roll then the axis is left unchanged
        }
    
        //the selected axis value is returned
        switch (axis) {
          case kX:    
            this.setGyroAngleX(angle);
            break;
          case kY:
            this.setGyroAngleY(angle);
            break;
          case kZ:
            this.setGyroAngleZ(angle);
            break;
          default:
        }
        
  }
  
  /**
   * Allow the gyro angle X to be set to a given value. 
   * This may happen with unread values in the 
   * buffer, it is suggested that the IMU is not 
   * moving when this method is run.
   * 
   * @param angle a double in degrees (CCW positive)
   */
  public void setGyroAngleX(double angle) {
    synchronized (this) {
      m_delta_ang_x = angle;
    }
  }

  /**
   * Allow the gyro angle Y to be set to a given value. 
   * This may happen with unread values in the 
   * buffer, it is suggested that the IMU is not 
   * moving when this method is run.
   * 
   * @param angle a double in degrees (CCW positive)
   */
  public void setGyroAngleY(double angle) {
    synchronized (this) {
      m_delta_ang_y = angle;
    }
  }

  /**
   * Allow the gyro angle Z to be set to a given value. 
   * This may happen with unread values in the 
   * buffer, it is suggested that the IMU is not 
   * moving when this method is run.
   * 
   * @param angle a double in degrees (CCW positive)
   */
  public void setGyroAngleZ(double angle) {
    synchronized (this) {
      m_delta_ang_z = angle;
    }
  }

  /** 
   * @param axis the IMUAxis whose angle to return
   * @return the axis angle in degrees (CCW positive) 
   */
  public synchronized double getAngle(IMUAxis axis) {
    //if pitch, yaw or roll is inputed then it is replaced with its equivelant axis
    switch (axis) {
      case kYaw: 
        axis = m_yaw_axis;
        break;
      case kPitch: 
        axis = m_pitch_axis;
        break;
      case kRoll: 
        axis = m_roll_axis;
        break;
      default: //is it isn't pitch yaw or roll then the axis is left unchanged
    }

    //the selected axis value is returned
    switch (axis) {
      case kX:    
        if (m_simGyroAngleX != null) {
          return m_simGyroAngleX.get();
        }
        return m_angle_x_deg;
      case kY:
        if (m_simGyroAngleY != null) {
          return m_simGyroAngleY.get();
        }
        return m_angle_y_deg;
      case kZ:
        if (m_simGyroAngleZ != null) {
          return m_simGyroAngleZ.get();
        }
        return m_angle_z_deg;
        default:
    }
    //This return should never be reached.
    return 0.0;
  }

  /**
   * @return Acceleration of the X axis (m/sec^2)
   */
  public double getAccelerationX() {
    if (m_simAccelX != null) {
      return m_simAccelX.get();
    }
    return m_accel_x[0];
  }

  /**
   * @return Acceleration of the Y axis (m/sec^2)
   */
  public double getAccelerationY() {
    if (m_simAccelY != null) {
      return m_simAccelY.get();
    }
    return m_accel_y[0];
  }
  
  /**
   * @return Acceleration of the Z axis (m/sec^2)
   */
  public double getAccelerationZ() {
    if (m_simAccelZ != null) {
      return m_simAccelZ.get();
    }
    return m_accel_z[0];
  }

  /** 
   * Returns which axis, kX, kY, 
   * or kZ is set to the Yaw axis
   * 
   * @return IMUAxis Yaw Axis 
   */
  public IMUAxis getYawAxis() {
    return m_yaw_axis;
  }

  /** 
   * Returns which axis, kX, kY, 
   * or kZ is set to the Pitch axis
   * 
   * @return IMUAxis Pitch Axis 
   */
  public IMUAxis getPitchAxis() {
    return m_pitch_axis;
  }

  /** 
   * Returns which axis, kX, kY, 
   * or kZ is set to the Roll axis
   * 
   * @return IMUAxis Roll Axis 
   */
  public IMUAxis getRollAxis() {
    return m_roll_axis;
  }

  /**
   * Get the SPI port number.
   *
   * @return The SPI port number.
   */
  public int getPort() {
    return m_spi_port.value;
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", () -> getAngle(m_yaw_axis), null);
  }
}