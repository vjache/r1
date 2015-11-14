// Lib for sonar
#include <NewPing.h>
// Lib for I²C
#include <Wire.h>
// Lib for IMU
#include <troyka-imu.h>

#include <math.h>

#define PINGS_ARRAY_SIZE 12
#define SLOT_DEGREES (360 / PINGS_ARRAY_SIZE)

#define ROT_SPEED 110

typedef int array_pings[PINGS_ARRAY_SIZE];
/////////////////////////////////////////////////////////////////////////////

class ChassisDev {
    const int SPEED_LFT = 5;
    const int DIR_LFT   = 4;
    const int SPEED_RGT = 6;
    const int DIR_RGT   = 7;
  public:
    static void setup() {
      for (int i = 4; i < 8; i++)
        pinMode(i, OUTPUT);
    }
  public:
    void stop() {
      setSpeed(0, 0);
    }
    void rotate(int s) {
      setSpeed(s, -s);
    }
    void move(int s) {
      setSpeed(s, s);
    }
  private:
    void setSpeed(int lft, int rgt) {
      
        if (lft < 0)
          digitalWrite(DIR_LFT, LOW);
        else
          digitalWrite(DIR_LFT, HIGH);

        if (rgt < 0)
          digitalWrite(DIR_RGT, LOW);
        else
          digitalWrite(DIR_RGT, HIGH);

        analogWrite(SPEED_LFT, abs(lft));
        analogWrite(SPEED_RGT, abs(rgt));
       
    }
};

class SonarDev {
    NewPing _sonar  = NewPing(12, 10, 400);
  public:
    static void setup() {
      pinMode(12, OUTPUT);
      pinMode(10, INPUT);
    }
  public:
    int distance() {
      int uS = _sonar.ping();
      return uS / US_ROUNDTRIP_CM;
    }
};

class CompassDev {
    Compass compass;

    bool _calibrated = false;

    float x_min = 10000.0;
    float x_max = 0.0;
    float x_c;
    float x_r;

    float y_min = 10000.0;
    float y_max = 0.0;
    float y_c;
    float y_r;

    void resetCalibrationParameters() {
      x_min = 10000.0;
      x_max = 0.0;
      x_c = 0;
      x_r = 0;

      y_min = 10000.0;
      y_max = 0.0;
      y_c = 0;
      y_r = 0;

      _calibrated = false;
    }
  public:
    static void setup() {}
    CompassDev() {
      compass.begin();
      compass.setRange(RANGE_16);
    }
  public:
    int azimuth() {
      compass.readXYZ_Calib();

      float dx = (compass.readX_Gauss() - x_c) / x_r;
      float dy = (compass.readY_Gauss() - y_c) / y_r;

      float azim = atan2(dy, dx) * 180 / PI;

      // Convert azimuth from [-179, 180] to [0, 359]
      if (azim < 0)
        azim = 360 + azim;

      return (int) azim;
    }
    void calibrate(int timeout) {
      resetCalibrationParameters();
      const int dt = 50;
      while (timeout > 0)
      {
        compass.readXYZ_Calib();

        float x = compass.readX_Gauss();
        float y = compass.readY_Gauss();

        if (x > x_max)
          x_max = x;
        if (x < x_min)
          x_min = x;

        if (y > y_max)
          y_max = y;
        if (y < y_min)
          y_min = y;

        delay(dt);
        timeout -= dt;
      }

      x_c = (x_min + x_max) / 2;
      y_c = (y_min + y_max) / 2;
      x_r = abs(x_max - x_min) / 2;
      y_r = abs(y_max - y_min) / 2;

      _calibrated = true;
    }
};

class Log {
  private:
    bool _on = true;
  public:
    Log(bool on): _on(on) {}
    static void setup() {
      Serial.begin(9600);
    }
    void print(String s) {
      if (_on) Serial.print(s);
    }
    void print(int s) {
      if (_on) Serial.print(s);
    }
    void print(float s) {
      if (_on) Serial.print(s);
    }
    void println(String s) {
      if (_on) Serial.println(s);
    }
    void println(int s) {
      if (_on) Serial.println(s);
    }
    void println(float s) {
      if (_on) Serial.println(s);
    }

    void println(int i1, String d, int i2 ) {
      print(i1);
      print(d);
      println(i2);
    }

    void println(String d, int i2 ) {
      print(d);
      println(i2);
    }

    void print_array_pings(String title, array_pings& arr) {
      if (_on) {
        Serial.print(title);
        Serial.print(": [");
        for (int i = 0; i < PINGS_ARRAY_SIZE; i++) {
          if (i > 0)
            Serial.print(", ");
          Serial.print(arr[i]);
        }
        Serial.println("]");
      }
    }
};

class Algorithm {
    ChassisDev chassis;
    CompassDev compass;
    SonarDev   sonar;
    Log _log = Log(false);
  protected:
    void scan_dirs(int timeout, array_pings& pings) {
      array_pings counts;
      reset_array_pings(counts);
      const int dt = 50;
      while (timeout > 0)
      {
        int d = sonar.distance();
        int s = compass.azimuth() / SLOT_DEGREES;
        _log.println(s, ":", d);
        pings[s] += d;
        counts[s] ++;
        delay(dt);
        timeout -= dt;
      }

      _log.print_array_pings("Counts", counts);
      _log.print_array_pings("Pings", pings);

      for (int i = 0; i < PINGS_ARRAY_SIZE; i++) {
        pings[i] = pings[i] / (counts[i] == 0 ? 1 : counts[i]);
      }
    };

    void move() {

      while (true) {
        int d = sonar.distance();

        if (d > 200)
          chassis.move(180);
        else if (d > 100)
          chassis.move(150);
        else if (d > 80)
          chassis.move(120);
        else if (d > 30)
          chassis.move(110);
        else break;

        delay(50);
      }

      chassis.stop();
    };

    void turnTo(int s) {
      chassis.rotate(ROT_SPEED);

      while ((compass.azimuth() / SLOT_DEGREES) != s)
        delay(50);

      _log.println("Direction reached: ", s);

      chassis.stop();
    }
    /**
     * If success then returns an azimuth slot value
     * from range [0..11], otherwise -1.
     */
    int chooseDir(array_pings& pings) {
      int dmax = 0;
      int dir = -1;
      for (int i = 0; i < PINGS_ARRAY_SIZE; i++) {
        int di = pings[i];
        if (di > dmax)
        {
          dmax = di;
          dir = i;
        }
      }
      return dir;
    }
  public:
    static void setup() {
      ChassisDev::setup();
      CompassDev::setup();
      SonarDev::setup();
    }
    static void reset_array_pings(array_pings& arr) {
      for (int i = 0; i < PINGS_ARRAY_SIZE; i++)
        arr[i] = 0;
    }

    void run() {

      _log.println("0. Started");
      // Lets begin.
      // 1. Sleep for a while, zzzzz...
      chassis.stop();
      delay(15000);
      // 2. Start rotation
      chassis.rotate(ROT_SPEED);
      // 3. Calibrate compass while rotating
      _log.println("1. Calibrating.");
      compass.calibrate(10000);
      _log.println("2. Calibrated.");
      chassis.stop();
      delay(1000);
      for (int i = 0; i < 20; i++) {
        array_pings pings;
        reset_array_pings(pings);
        // 4.
        _log.println("3. Scaning.");
        chassis.rotate(ROT_SPEED);
        // 5. While rotating use sonar to scan obstacle distances
        scan_dirs(3000, pings);
        _log.print_array_pings("3.1. Scanned", pings);
        // 6. Stop!
        chassis.stop();
        delay(1000);
        // 7. Choose direction to move
        int dirSlot = chooseDir(pings);
        _log.println("4. Choose dir:", dirSlot);
        if (dirSlot == -1)
          return;
        // 8. Turn
        _log.println("5. Turning.");
        turnTo(dirSlot);
        // 9. Move
        _log.println("5. Moving!");
        move();
      }
    }
};

void setup() {
  //Log::setup();
  Algorithm::setup();
}

void loop() {
  Algorithm a;
  a.run();
}



