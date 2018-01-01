#include <math.h>

class Effect {
  protected:
    unsigned long start_time_us, display_time_us;
    double start_progress;
  public:
    Effect(unsigned long _display_time_us) {
      display_time_us = _display_time_us;
    };
    ~Effect() {};
    void init(double, unsigned long );
    virtual int run(double, unsigned long);
    bool is_finished(unsigned long);
};

void Effect::init(double _start_progress, unsigned long _start_time_us) {
  start_time_us = _start_time_us;
  start_progress = _start_progress;
}

bool Effect::is_finished(unsigned long current_time_us) {
  return current_time_us >= (display_time_us + start_time_us);
}

class BreatheEffect: public Effect {
  public:
    BreatheEffect(unsigned long _display_time_us, int _time_scale_factor) : Effect(_display_time_us) {
      time_scale_factor = _time_scale_factor;
    }
    int run (double, unsigned long);
  private:
    int time_scale_factor;
};

int BreatheEffect::run(double cur_progress, unsigned long cur_time_us) {
  // http://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
  double cur_time_ms = (cur_time_us - start_time_us) / 1000.0;
  return 255.0 - (exp(sin(cur_time_ms/((double) time_scale_factor)*PI)) - 0.36787944)*108.0;
}

class AllOnEffect: public Effect {
  public:
    AllOnEffect(unsigned long _display_time_us) : Effect(_display_time_us) {}
    int run(double, unsigned long);
};

int AllOnEffect::run(double cur_progress, unsigned long cur_time_us) {
  //Serial.println(cur_progress);
  return 255;
}

class HalfDiskEffect: public Effect {
  public:
    HalfDiskEffect(unsigned long _display_time_us) : Effect(_display_time_us) {}
    int run(double, unsigned long);
};

int HalfDiskEffect::run(double cur_progress, unsigned long cur_time_us) {
  if (cur_progress < 0.5) {
    return 255;
  } else {
    return 0;
  }
}

unsigned long last_time_us;
const int TTL_PORT = 10;
int rotations_per_sec = 30;
unsigned long rotation_time_us = (1000L * 1000L) / ((unsigned long) rotations_per_sec);

const unsigned long one_sec_us = 1000L * 1000L;
AllOnEffect all_on_3(3L * one_sec_us);
BreatheEffect breathe(13.4 * one_sec_us, 2000);
HalfDiskEffect half_disk(10000L * one_sec_us);

//Effect * effects_3d[] = {&half_disk};
Effect * effects[] = {&all_on_3, &breathe};
int current_effect = -1;
int effect_cnt = sizeof(effects)/sizeof(effects[0]);

void setup()
{
  last_time_us = micros();
  pinMode(TTL_PORT, OUTPUT);
  //Serial.begin(9600);
  //Serial.print("Rotation time microseconds: ");
  //Serial.println(rotation_time_us);
}

void loop() {
  unsigned long cur_time_us = micros();
  if (cur_time_us < last_time_us) {
    current_effect = -1; // overflow, start over
  }
  last_time_us = cur_time_us;
  double cur_progress = ((double) (cur_time_us % rotation_time_us)) / ((double) rotation_time_us);
  if (current_effect < 0 || effects[current_effect]->is_finished(cur_time_us)) {
    current_effect = (current_effect + 1) % effect_cnt;
    effects[current_effect]->init(cur_progress, cur_time_us);
  }
  int val = effects[current_effect]->run(cur_progress, cur_time_us);
  analogWrite(TTL_PORT, val);
}

