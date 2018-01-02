#include <math.h>

const int TTL_PORT = 10;
const bool DEBUG = true;

/* CLASS SHIT */

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

class Timeline {
  Effect **effects;
  int current_effect;
  int effect_cnt;
  public:
    Timeline(Effect **_effects, int _effect_cnt) {
      effects = _effects;
      current_effect = -1;
      effect_cnt = _effect_cnt;
    };
    ~Timeline() {};
    int run(double, unsigned long);
    void reset() {
      current_effect = -1;
    }
};

int Timeline::run(double cur_progress, unsigned long cur_time_us) {
  if (current_effect < 0 || effects[current_effect]->is_finished(cur_time_us)) {
    current_effect = (current_effect + 1) % effect_cnt;
    effects[current_effect]->init(cur_progress, cur_time_us);
  }
  return effects[current_effect]->run(cur_progress, cur_time_us);
}

/* 1D effects */

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
  return 255;
}

/* 2D effects */

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

/* FRAMERATE ADJUSTMENT */

double rotations_per_sec = 30.0;
unsigned long rotation_time_us = 0L;
int last_dial_position = 0;

void adjust_rps(double delta) {
  rotations_per_sec += delta;
  if (DEBUG) {
    Serial.print("rotations per second: ");
    Serial.println(rotations_per_sec);  
  }
  rotation_time_us = ceil((1000.0 * 1000.0) / rotations_per_sec);
}

void check_for_rps_adjustment() {
  int dial_position = analogRead(A0);
  if (dial_position != last_dial_position) {
    double delta = last_dial_position - dial_position;
    adjust_rps(delta / 100);
    last_dial_position = dial_position;
  }  
}

/* TIMELINE DEFINITIONS */

const unsigned long one_sec_us = 1000L * 1000L;
AllOnEffect all_on_3(3L * one_sec_us);
BreatheEffect breathe(13.4 * one_sec_us, 2000);
HalfDiskEffect half_disk(10000L * one_sec_us);

Effect * effects_2d[] = {&all_on_3, &breathe};
Timeline timeline_2d(effects_2d, sizeof(effects_2d)/sizeof(effects_2d[0]));
Effect * effects_3d[] = {&half_disk};
Timeline timeline_3d(effects_3d, sizeof(effects_3d)/sizeof(effects_3d[0]));

int current_timeline = -1;
Timeline* registered_timelines[] = {&timeline_2d, &timeline_3d};
int show_cnt = sizeof(registered_timelines)/sizeof(registered_timelines[0]);

/* TIMELINE SWITCHING */

bool is_timeline_change_requested = false;

/* NORMAL ARDUINO FUNCTIONS */

unsigned long last_time_us;
unsigned int loop_counter = 0;

void setup() {
  last_time_us = micros();
  pinMode(TTL_PORT, OUTPUT);
  adjust_rps(0); // initialize rotation_time_us
  last_dial_position = analogRead(A0);
  if (DEBUG) {
    Serial.begin(9600);  
  }
}

void loop() {
  if (loop_counter % 1024 == 0) {
    // 10 orders of magnitude less frequent than main loop
    check_for_rps_adjustment();
  }
  if (current_timeline < 0 || is_timeline_change_requested) {
    current_timeline = (current_timeline + 1) % show_cnt;
  }
  unsigned long cur_time_us = micros();
  if (cur_time_us < last_time_us) {
    registered_timelines[current_timeline]->reset(); // time overflow, start show over
  }
  last_time_us = cur_time_us;
  double cur_progress = ((double) (cur_time_us % rotation_time_us)) / ((double) rotation_time_us);
  int val = registered_timelines[current_timeline]->run(cur_progress, cur_time_us); 
  analogWrite(TTL_PORT, val);
  loop_counter++;
}


