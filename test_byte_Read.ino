#include "sensors_pos.h"
#include "libio.h"

// --- Constants ---
const short base_speed = 200;
const long kp = 45;
const long kd = 500;
const long kd_max=80;
const long kp_max=255;

// Time Constants (T) in microseconds
// Adjust these to change how "heavy" or "fast" the smoothing feels
const float T_DDT = 25000.0;      // Smoothing for the derivative
const float T_DDTA=  25000.0;    // Secondary smoothing for ddta

// --- State Variables ---
unsigned long old_frame_time = 0;
long old1_line_pos = 0;
unsigned long old1_line_time = 0;
long old2_line_pos = 0;
unsigned long old2_line_time = 0;

float ddt = 0;
float ddta = 0;

long last_good_line = 0;
unsigned long lost_time = 0;

// Intersection & 90-degree logic
short count_intersection = 0;
uint16_t mask = 0;
unsigned long ignore_90 = 0;
bool ready_for90_l = 0, ready_for90_r = 0;
unsigned long ready_for90_l_time = 0, ready_for90_r_time = 0;
const long time90 = 200000L;

#define smooth(a, b, T, dt) do { float alfa = (float)dt / ((float)T + (float)dt); a += (b - a) * alfa; } while(0)

inline uint16_t get_sensors() {
  uint8_t sensors_l = *portInputRegister(3);
  uint8_t sensors_h = *portInputRegister(12);
  return (sensors_l << 3) | ((sensors_h >> 5) & 0x7);
}

// const long MASK_HISTORY_SIZE = 10;
// uint16_t mask_history[MASK_HISTORY_SIZE];
// uint8_t bit_counts[16] = {0};
// long mask_index = 0;
// uint16_t sliding_mask = 0;
// void update_sliding_mask(uint16_t current_sensors) {
//     // 1. Get the oldest sample
//     uint16_t old_sensors = mask_history[mask_index];
    
//     // 2. Identify which bits changed between the oldest and the newest
//     uint16_t changed = current_sensors ^ old_sensors;

//     // 3. Only loop through bits that actually changed
//     if (changed > 0) {
//         for (int i = 0; i < 16; i++) {
//             // Check if this specific bit changed
//             if (changed & (1 << i)) {
//                 // If it's 1 now, it was 0 in old_sensors (Increment)
//                 if (current_sensors & (1 << i)) {
//                     bit_counts[i]++;
//                     sliding_mask |= (1 << i); // Set bit in mask
//                 } 
//                 // If it's 0 now, it was 1 in old_sensors (Decrement)
//                 else {
//                     bit_counts[i]--;
//                     if (bit_counts[i] <= 0) {
//                         bit_counts[i] = 0; // Safety clamp
//                         sliding_mask &= ~(1 << i); // Clear bit in mask
//                     }
//                 }
//             }
//         }
//     }

//     // 4. Update the history buffer
//     mask_history[mask_index] = current_sensors;
//     mask_index = (mask_index + 1) % MASK_HISTORY_SIZE;
// }

bool get_ready_left(uint16_t sensors)  { return __builtin_popcount(sensors & 0b00000111111) > 3; }
bool get_ready_right(uint16_t sensors) { return __builtin_popcount(sensors & 0b11111100000) > 3; }

void setup() {
  Serial.begin(115200);
  motors::setup();
}
float cur_speed_left=0;
float cur_speed_right=0;
float target_speed_right=0;
float target_speed_left=0;
void set_left_speed(long speed){
  target_speed_left=speed;
}
void set_right_speed(long speed){
  target_speed_right=speed;
}
void update_motors(long dt){
  smooth(cur_speed_right,target_speed_right,30000,dt);
  smooth(cur_speed_left,target_speed_left,30000,dt);
  motors::move_right(cur_speed_right);
  motors::move_left (cur_speed_left);
}
void move(long l,long r,long unt){
  while (micros()<unt){
    long new_frame_time = micros();
    long d_t = new_frame_time - old_frame_time;
    old_frame_time = new_frame_time;
    set_left_speed(l); set_right_speed(r);
    update_motors(d_t);
    while(micros()<new_frame_time+550);
  }
}
void mstop(){
  while (1){
    long new_frame_time = micros();
    long d_t = new_frame_time - old_frame_time;
    old_frame_time = new_frame_time;
    set_left_speed(0); set_right_speed(0);
    update_motors(d_t);
    while(micros()<new_frame_time+550);
  }
}
void loop() {
  uint16_t sensors = get_sensors();
  // update_sliding_mask(sensors);
  short count = __builtin_popcount(sensors);
  // Serial.print(__builtin_ctz(sensors));
  // Serial.print(" ");
  // Serial.print(count);
  // Serial.print(" ");
  // Serial.println(sensors,2);
  // return;
  unsigned long new_frame_time = micros();
  unsigned long d_t = new_frame_time - old_frame_time;
  old_frame_time = new_frame_time;
  update_motors(d_t);
  static long cnt=0;
  static long cntt=0;
  // if(cnt++==50){cnt=0;
  //   // Serial.print("255 -255 ");
  //   // Serial.print(constrain(ddta * kd, -1000, 10000));
  //   // Serial.print(" ");
  //   // Serial.println(last_good_line * kp);
  //   // Serial.println(new_frame_time-cntt);
  //   Serial.print(__builtin_ctz(sensors));
  //   Serial.print(" ");
  //   Serial.print(count);
  //   Serial.print(" ");
  //   Serial.println(sensors,2);
  //   // Serial.print(cur_speed_left);
  //   // Serial.print(" ");
  //   // Serial.println(cur_speed_right);
  //   // if(count!=0){
  //   // Serial.print(__builtin_popcount(sliding_mask & 0b00000011111));
  //   // Serial.print(" ");
  //   // Serial.print(__builtin_popcount(sliding_mask & 0b11111000000));
  //   // Serial.print(" ");
  //   // Serial.println(sliding_mask,2);
  //   // // Serial.println(cur_speed_right);
  //   // cntt=new_frame_time;}
  // }
  // if (d_t == 0) return; // Prevent division by zero

  // --- 90 Degree & Intersection Logic ---
  if (new_frame_time > ignore_90) {
    if (get_ready_left(sensors) && !ready_for90_l) {
      ready_for90_l = 1;
      ready_for90_l_time = new_frame_time;
      // motors::move_left (0);
      // motors::move_right(0);
      // while(1);
    }
    if (get_ready_right(sensors) && !ready_for90_r) {
      ready_for90_r = 1;
      ready_for90_r_time = new_frame_time;
    }
    if (ready_for90_l || ready_for90_r) mask |= sensors;

    if (__builtin_popcount(mask &0b00000111111)>2 && __builtin_popcount(mask &0b11111100000)>2 ) {
      count_intersection++;
      ignore_90 = new_frame_time + 500000; // 0.6s ignore
      // memset(mask_history, 0, sizeof(mask_history));
      // sliding_mask=0;
      ready_for90_r = ready_for90_l = 0;
      mask = 0;
      if (new_frame_time>30000000L) {
        mstop();
      }
    }
    
    // Timestep-independent turn execution
    if (ready_for90_l && (new_frame_time - ready_for90_l_time > 300000)) {
      ready_for90_r = ready_for90_l = 0; mask = 0;
      // stop();
      move(-200,200,new_frame_time+time90);
      return;
    } else if (ready_for90_r && (new_frame_time - ready_for90_r_time > 300000)) {
      ready_for90_r = ready_for90_l = 0; mask = 0;
      // stop();
      move(200,-200,new_frame_time+time90);
      return;
    }
  }

  // --- Line Calculations ---
  if (count == 0) {
    // if( last_good_line>=-3 &&last_good_line<=3){
    
    // }
    // else{
      // Search logic when line is lost
      float dt_lost = (new_frame_time - lost_time) / 1000000.0f;
      float search_factor = 0.4f + min(dt_lost, 0.6f);
      long turn = (last_good_line * kp) * search_factor;

      set_left_speed(constrain(base_speed + turn, -60, 200));
      set_right_speed(constrain(base_speed - turn, -60, 200));
    // }
    smooth(ddt, 0.0f, T_DDT, d_t);
  }
  else{
    // Line found: calculate position
    long line_pos = sensors_pos[sensors] / count;
    last_good_line = line_pos;
    lost_time = new_frame_time;

    // Timestep independent derivative calculation
    if (line_pos != old1_line_pos) {
      old2_line_pos = old1_line_pos;
      old1_line_pos = line_pos;
      old2_line_time = old1_line_time;
      old1_line_time = new_frame_time;
      
      float instant_ddt = ((float)(old1_line_pos - old2_line_pos) * 1e6f) / (old1_line_time - old2_line_time);
      constrain(instant_ddt,-255*kd,255*kd);
      smooth(ddt, instant_ddt, T_DDT, d_t);
    } else {
      // If position hasn't changed, decay ddt towards zero based on time
      smooth(ddt, 0.0f, T_DDT, d_t);
    }

    // Secondary smoothing for derivative

    // --- PID Output ---

    long delta = constrain(line_pos * kp,-kp_max,kp_max) + constrain(ddta * kd, -kd_max, kd_max);
    
    // Timestep independent Integral/Accumulator


    // Dynamic Speed
    float speed_drop = constrain(1.0f - (abs(line_pos) / 50.0f), 0.3f, 1.0f);
    long current_base = base_speed * speed_drop;

    set_left_speed(constrain(current_base + delta, -60, 200));
    set_right_speed(constrain(current_base - delta, -60, 200));
  }

  smooth(ddta, ddt, T_DDTA, d_t);
  // while(micros()-new_frame_time<550);
}