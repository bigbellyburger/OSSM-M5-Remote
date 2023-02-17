#pragma GCC optimize ("Ofast")
#include <M5Core2.h>
#include <AXP192.h>
#include <ESP32Encoder.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PatternMath.h>
#include "OneButton.h"          //For Button Debounce and Longpress
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <SPI.h>
#include "m5_messages.h"
#include "ui/ui.h"
#include <EEPROM.h>
#include "main.h"

int screenWidth  = 320;
int screenHeight = 240;

///////////////////////////////////////////
////
////  To Debug or not to Debug
////
///////////////////////////////////////////

// Uncomment the following line if you wish to print DEBUG info
#define DEBUG 

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebug(...) ((void)0)
#define LogDebugFormatted(...) ((void)0)
#endif

// #define OFF 0.0
// #define ON 1.0

// Screens 

#define ST_UI_START 0
#define ST_UI_HOME 1

#define ST_UI_MENUE 10
#define ST_UI_PATTERN 11
#define ST_UI_Torqe 12
#define ST_UI_EJECTSETTINGS 13

#define ST_UI_SETTINGS 20

int st_screens = ST_UI_START;



// Menü States

#define CONNECT 0
#define HOME 1
#define MENUE 2
#define MENUE2 3
#define TORQE 4
#define PATTERN_MENUE 5
#define PATTERN_MENUE2 6
#define PATTERN_MENUE3 7
#define CUM_MENUE 20

int menuestatus = CONNECT;

// EEPROM Area:

#define EJECT 0
#define DARKMODE 1
#define VIBRATE 2
#define LEFTY 6

bool eject_status = false;
bool dark_mode = false;
bool vibrate_mode = true;
bool touch_home = false;
bool touch_disabled = false;

// // Command States
// #define CONN 0
// #define SPEED 1
// #define DEPTH 2
// #define STROKE 3
// #define SENSATION 4
// #define PATTERN 5
// #define TORQE_F 6
// #define TORQE_R 7
// #define OFF 10
// #define ON  11
// #define SETUP_D_I 12
// #define SETUP_D_I_F 13
// #define REBOOT 14

// #define CUMSPEED 20
// #define CUMTIME 21
// #define CUMSIZE   22
// #define CUMACCEL  23

// #define CONNECT 88
// #define HEARTBEAT 99

int displaywidth;
int displayheight;
int progheight = 30;
int distheight = 10;
// int S1Pos;
// int S2Pos;
// int S3Pos;
// int S4Pos;
// bool rstate = false;
// int pattern = 2;
char patternstr[20];
// bool onoff = false;


long speedenc = 0;
long depthenc = 0;
long strokeenc = 0;
long sensationenc = 0;
long torqe_f_enc = 0;
long torqe_r_enc = 0;
// long cum_t_enc = 0;
// long cum_si_enc =0;
// long cum_s_enc = 0;
// long cum_a_enc = 0;
long encoder4_enc = 0;

// extern float maxdepthinmm = 180.0;
// extern float speedlimit = 1200;
int speedscale = -5;

// float speed = 0.0;
// float depth = 0.0;
// float stroke = 0.0;
// float sensation = 0.0;
// float torqe_f = 100.0;
// float torqe_r = -180.0;
// float cum_time = 0.0;
// float cum_speed = 0.0;
// float cum_size = 0.0;
// float cum_accel = 0.0;

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

extern ossm_state_t want_ossm_state = {
  .speed = 0,
  .depth = 0,
  .stroke = 0,
  .sensation = 0,
  .pattern = 2,
  .torque_f = 100.0,
  .torque_r = -180.0,
  .on = false,
};
extern ossm_broadcast_state_t known_ossm = {
  .maxspeed = 1200,
  .maxdepth = 180,
  .state = want_ossm_state,
};

unsigned long last_control_sent;
bool first_connect;
bool ossm_connected;
cum_state_t want_cum_state;
cum_state_t known_cum_state;

// // Variable to store if sending data was successful
// String success;
// 
// float out_esp_speed;
// float out_esp_depth;
// float out_esp_stroke;
// float out_esp_sensation;
// float out_esp_pattern;
// bool out_esp_rstate;
// bool out_esp_connected;
// int out_esp_command;
// float out_esp_value;
// int out_esp_target;

// float incoming_esp_speed;
// float incoming_esp_depth;
// float incoming_esp_stroke;
// float incoming_esp_sensation;
// float incoming_esp_pattern;
// bool incoming_esp_rstate;
// bool incoming_esp_connected;
// bool incoming_esp_heartbeat;
// int incoming_esp_target;

// typedef struct struct_message {
//   float esp_speed;
//   float esp_depth;
//   float esp_stroke;
//   float esp_sensation;
//   float esp_pattern;
//   bool esp_rstate;
//   bool esp_connected;
//   bool esp_heartbeat;
//   int esp_command;
//   float esp_value;
//   int esp_target;
// } struct_message;

// bool esp_connect = false;
// bool m5_first_connect = false;

// int lastControlSent = 0;
// struct_message outgoingcontrol;
// struct_message incomingcontrol;

esp_now_peer_info_t peerInfo;

// unsigned long Heartbeat_Time = 0;
// const long Heartbeat_Interval = 10000;

// Bool

// bool EJECT_On = false;
// bool OSSM_On = false;

#define EEPROM_SIZE 200

// Tasks:

TaskHandle_t eRemote_t  = nullptr;  // Esp Now Remote Task

void espNowRemoteTask(); // Handels the EspNow Remote
bool connectbtn(); //Handels Connectbtn
int64_t touchmenue();
void vibrate();
void mxclick();
bool mxclick_short_waspressed = false;
void mxlong();
bool mxclick_long_waspressed = false;
void click2();
bool click2_short_waspressed = false;
void click3();
bool click3_short_waspressed = false;

// init the tft espi
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;  // Descriptor of a display driver
static lv_indev_drv_t indev_drv; // Descriptor of a touch driver

M5Display *tft;
static lv_obj_t * kb;

void tft_lv_initialization() {
  M5.begin();
  lv_init();
  static lv_color_t buf1[(LV_HOR_RES_MAX * LV_VER_RES_MAX) / 10];  // Declare a buffer for 1/10 screen siz
  static lv_color_t buf2[(LV_HOR_RES_MAX * LV_VER_RES_MAX) / 10];  // second buffer is optionnal

  // Initialize `disp_buf` display buffer with the buffer(s).
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, (LV_HOR_RES_MAX * LV_VER_RES_MAX) / 10);

  tft = &M5.Lcd;
}

// Display flushing
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft->startWrite();
  tft->setAddrWindow(area->x1, area->y1, w, h);
  tft->pushColors((uint16_t *)&color_p->full, w * h, true);
  tft->endWrite();

  lv_disp_flush_ready(disp);
}

void init_disp_driver() {
  lv_disp_drv_init(&disp_drv);  // Basic initialization

  disp_drv.flush_cb = my_disp_flush;  // Set your driver function
  disp_drv.draw_buf = &draw_buf;      // Assign the buffer to the display
  disp_drv.hor_res = LV_HOR_RES_MAX;  // Set the horizontal resolution of the display
  disp_drv.ver_res = LV_VER_RES_MAX;  // Set the vertical resolution of the display

  lv_disp_drv_register(&disp_drv);                   // Finally register the driver
  lv_disp_set_bg_color(NULL, lv_color_hex3(0x000));  // Set default background color to black
}

void my_touchpad_read(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
  if(touch_disabled == false){
  TouchPoint_t pos = M5.Touch.getPressPoint();
  bool touched = ( pos.x == -1 ) ? false : true;  

  if(!touched) {
      data->state = LV_INDEV_STATE_RELEASED;
  } else {
    if (M5.BtnA.wasPressed()){  // tab 1 : A Button
      LogDebug("ButtonA");
      data->point.x = 80; data->point.y = 220; // mouse position x,y
      data->state =LV_INDEV_STATE_PR; M5.update();
      } else if (M5.BtnB.wasPressed()){  // tab 2 : B Button
      LogDebug("ButtonB");
      data->point.x = 160; data->point.y = 220;
      data->state =LV_INDEV_STATE_PR; M5.update();
      } else if (M5.BtnC.wasPressed()){  // tab 3 : C Button
      LogDebug("ButtonC");
      data->point.x = 270; data->point.y = 220;
      data->state =LV_INDEV_STATE_PR; M5.update();
      } else {
    data->state = LV_INDEV_STATE_PRESSED; 
    data->point.x = pos.x;
    data->point.y = pos.y;
  }
  } 
}
}

void init_touch_driver() {
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);  // register
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void print_ossm_state(ossm_state_t *state) {
  Serial.printf("speed=%f depth=%f stroke=%f sensation=%f pattern=%i torque_f=%f torque_r=%f on=%i\n",
    state->speed,
    state->depth,
    state->stroke,
    state->sensation,
    state->pattern,
    state->torque_f,
    state->torque_r,
    state->on ? 1 : 0
  );
}

void onReceiveOssmState() {
  if(known_ossm.state.on) {
    lv_obj_add_state(ui_HomeButtonM, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_HomeButtonM, LV_STATE_CHECKED);
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.printf("Received %i: ", len);
  for (int i = 0; i < len; i++)
  {
      Serial.printf("%02X", *(incomingData + i));
  }
  Serial.printf("\n");

  if(len < sizeof(m5_message_header_t)) {
    LogDebug("Received too short message");
    return;
  }

  m5_message_header_t incoming_header;

  memcpy(&incoming_header, incomingData, sizeof(m5_message_header_t));
  if(incoming_header.header != M5_HEADER) {
    LogDebug("Received message with invalid header");
    return;
  }

  if(incoming_header.target != 0 && incoming_header.target != M5_ID) {
    LogDebug("Received message for other target");
    return;
  }

  switch(incoming_header.message_type) {
    // case OSSM_HOMED:
    // {
    //   ossm_homed_t* ossm_homed = (ossm_homed_t*)(incomingData + bodyLength);
    //   Serial.printf("Received OSSM_HOMED message: maxdepth=%u maxspeed=%u\n", ossm_homed->maxspeed, ossm_homed->maxdepth);
    //   print_ossm_state(&ossm_homed->state);

    //   speedlimit = ossm_homed->maxspeed;
    //   maxdepthinmm = ossm_homed->maxdepth;
    //   known_ossm.state = ossm_homed->state;
    //   ossm_connected = true;
    //   onReceiveOssmState();
    //   break;
    // }
    case OSSM_BROADCAST_STATE:
    {
      ossm_broadcast_state_t* msg = (ossm_broadcast_state_t*)(incomingData + sizeof(m5_message_header_t));
      Serial.print("Received state: ");
      print_ossm_state(&msg->state);
      
      known_ossm.state = msg->state;
      ossm_connected = true;
      if(!first_connect) {
        first_connect = true;
        lv_label_set_text(ui_connect, "Connected");
        lv_scr_load_anim(ui_Home, LV_SCR_LOAD_ANIM_FADE_ON,20,0,false);
      }
      onReceiveOssmState();
      break;
    }
    case CUM_BROADCAST_STATE:
    {
      cum_state_t* cum_state = (cum_state_t*)(incomingData + sizeof(m5_message_header_t));
      known_cum_state = *cum_state;
      break;
    }
    default:
      Serial.printf("Received unknown message: %x\n", incoming_header.message_type);
      break;
  }

  // memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));

  // if(incomingcontrol.esp_target == M5_ID && incomingcontrol.esp_connected == true && m5_first_connect == false){
  //   speedlimit = incomingcontrol.esp_speed;
  //   maxdepthinmm = incomingcontrol.esp_depth;
  //   pattern = incomingcontrol.esp_pattern;
  //   outgoingcontrol.esp_target = OSSM_ID;
  //   esp_err_t result = esp_now_send(Broadcast_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
  //   LogDebug(result);
  //   if (result == ESP_OK) {
  //     m5_first_connect = true;
  //     lv_label_set_text(ui_connect, "Connected");
  //     lv_scr_load_anim(ui_Home, LV_SCR_LOAD_ANIM_FADE_ON,20,0,false);
  //   }
  // }
  // switch(incomingcontrol.esp_command)
  //   {
  //   case OFF: 
  //   {
  //   lv_obj_clear_state(ui_HomeButtonM, LV_STATE_CHECKED);
  //   OSSM_On = false;
  //   }
  //   break;
  //   case ON:
  //   {
  //   lv_obj_add_state(ui_HomeButtonM, LV_STATE_CHECKED);
  //   OSSM_On = true;
  //   }
  //   break;
  //   }
}

bool _send_message(m5_message_type message_type, unsigned short target, uint8_t *data, int len) {
  uint8_t buffer[sizeof(m5_message_header_t) + len];
  m5_message_header_t* header = (m5_message_header_t*) buffer;
  header->header = M5_HEADER;
  header->message_type = message_type;
  header->sender = M5_ID;
  header->target = target;
  memcpy(&buffer[sizeof(m5_message_header_t)], data, len);

  Serial.printf("Sent %i: ", sizeof(buffer));
  for (int i = 0; i < sizeof(buffer); i++)
  {
      Serial.printf("%02X", buffer[i]);
  }
  Serial.printf("\n");

  for(int i = 0; i < 3; i++) {
    esp_err_t result = esp_now_send(Broadcast_Address, (uint8_t *) &buffer, sizeof(buffer));
    if(result == ESP_OK) {
      return true;
    }

    delay(20);
  }

  return false;
}

bool send_connect(m5_connect_t* msg) {
  return _send_message(M5_CONNECT, OSSM_ID, (uint8_t*) msg, sizeof(*msg));
}

bool send_ossm_state() {
  if(!ossm_connected) return false;
  m5_set_ossm_state_t msg;
  msg.state = want_ossm_state;
  Serial.print("Sent state: ");
  print_ossm_state(&msg.state);
  return _send_message(M5_SET_OSSM_STATE, OSSM_ID, (uint8_t*) &msg, sizeof(msg));
}

bool send_setup_depth(m5_setup_depth_t* msg) {
  if(!ossm_connected) return false;
  return _send_message(M5_SETUP_DEPTH, OSSM_ID, (uint8_t*) msg, sizeof(*msg));
}

// //Sends Commands and Value to Remote device returns ture or false if sended
// bool SendCommand(int Command, float Value, int Target){
//   if(m5_first_connect == true) {
//     outgoingcontrol.esp_connected = true;
//     outgoingcontrol.esp_command = Command;
//     outgoingcontrol.esp_value = Value;
//     outgoingcontrol.esp_target = Target;
//     esp_err_t result = esp_now_send(Broadcast_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
//     if (result == ESP_OK) {
//       return true;
//     } else {
//       delay(20);
//       esp_err_t result = esp_now_send(Broadcast_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
//       return false;
//     }
//   }
// }

void connectbutton(lv_event_t * e)
{
  m5_connect_t msg;
  send_connect(&msg);

    // if(m5_first_connect == false){
    // outgoingcontrol.esp_command = HEARTBEAT;
    // outgoingcontrol.esp_heartbeat = true;
    // outgoingcontrol.esp_target = OSSM_ID;
    // esp_err_t result = esp_now_send(Broadcast_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    // }
}

void savesettings(lv_event_t * e)
{
	if(lv_obj_get_state(ui_ejectaddon) == 1){
		EEPROM.writeBool(EJECT,true);
	} else if(lv_obj_get_state(ui_ejectaddon) == 0){
		EEPROM.writeBool(EJECT,false);
	}
  if(lv_obj_get_state(ui_darkmode) == 1){
		EEPROM.writeBool(DARKMODE,true);
	} else if(lv_obj_get_state(ui_darkmode) == 0){
		EEPROM.writeBool(DARKMODE,false);
	}
  if(lv_obj_get_state(ui_vibrate) == 1){
		EEPROM.writeBool(VIBRATE,true);
	} else if(lv_obj_get_state(ui_vibrate) == 0){
		EEPROM.writeBool(VIBRATE,false);
	}
  if(lv_obj_get_state(ui_lefty) == 1){
		EEPROM.writeBool(LEFTY,true);
	} else if(lv_obj_get_state(ui_lefty) == 0){
		EEPROM.writeBool(LEFTY,false);
	}
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void screenmachine(lv_event_t * e)
{
  if (lv_scr_act() == ui_Start){
    st_screens = ST_UI_START;
  } else if (lv_scr_act() == ui_Home){
    st_screens = ST_UI_HOME;
    want_ossm_state.speed = lv_slider_get_value(ui_homespeedslider);
    speedenc =  fscale(0.5, known_ossm.maxspeed, 0, Encoder_MAP, want_ossm_state.speed, speedscale);
    encoder1.setCount(speedenc); 

    want_ossm_state.depth = lv_slider_get_value(ui_homedepthslider);       
    depthenc =  fscale(0, known_ossm.maxdepth, 0, Encoder_MAP, want_ossm_state.depth, 0);
    encoder2.setCount(depthenc);
            
    want_ossm_state.stroke = lv_slider_get_value(ui_homestrokeslider);    
    strokeenc =  fscale(0, known_ossm.maxdepth, 0, Encoder_MAP, want_ossm_state.stroke, 0);
    encoder3.setCount(strokeenc);

    want_ossm_state.sensation = lv_slider_get_value(ui_homesensationslider);
    sensationenc =  fscale(-100, 100, (Encoder_MAP/2*-1), (Encoder_MAP/2), want_ossm_state.sensation, 0);
    encoder4.setCount(sensationenc);        
            
  } else if (lv_scr_act() == ui_Menue){
    st_screens = ST_UI_MENUE;
  } else if (lv_scr_act() == ui_Pattern){
    st_screens = ST_UI_PATTERN;
  } else if (lv_scr_act() == ui_Torqe){
    st_screens = ST_UI_Torqe;
    want_ossm_state.torque_f = lv_slider_get_value(ui_outtroqeslider);
    torqe_f_enc = fscale(50, 200, 0, Encoder_MAP, want_ossm_state.torque_f, 0);
    encoder1.setCount(torqe_f_enc);

    want_ossm_state.torque_r = lv_slider_get_value(ui_introqeslider);
    torqe_r_enc = fscale(20, 200, 0, Encoder_MAP, want_ossm_state.torque_r, 0);
    encoder4.setCount(torqe_r_enc);

  } else if (lv_scr_act() == ui_EJECTSettings){
    st_screens = ST_UI_EJECTSETTINGS;
  } else if (lv_scr_act() == ui_Settings){
    st_screens = ST_UI_SETTINGS;
  }
}

void ejectcreampie(lv_event_t * e){
  if(want_cum_state.on == true){
    lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);
    want_cum_state.on = false;
  } else if(want_cum_state.on == false){
    lv_obj_add_state(ui_HomeButtonL, LV_STATE_CHECKED);
    want_cum_state.on = true;
  } 
}

void savepattern(lv_event_t * e){
  want_ossm_state.pattern = lv_roller_get_selected(ui_PatternS);
  lv_roller_get_selected_str(ui_PatternS,patternstr,0);
  lv_label_set_text(ui_HomePatternLabel,patternstr);
  LogDebug(want_ossm_state.pattern);
  send_ossm_state();
}

void homebuttonmevent(lv_event_t * e){
  LogDebug("HomeButton");
  want_ossm_state.on = !known_ossm.state.on;
  send_ossm_state();
}

void setupDepthInter(lv_event_t * e){
  m5_setup_depth_t msg;
  msg.fancy = false;
  send_setup_depth(&msg);
    // SendCommand(SETUP_D_I, 0.0, OSSM_ID);
}

void setupdepthF(lv_event_t * e){
  m5_setup_depth_t msg;
  msg.fancy = true;
  send_setup_depth(&msg);
    // SendCommand(SETUP_D_I_F, 0.0, OSSM_ID);
}

void setup(){
  M5.begin(true, false, true, true); //Init M5Core2.
  EEPROM.begin(EEPROM_SIZE);

  M5.Axp.SetCHGCurrent(AXP192::BATTERY_CHARGE_CURRENT);
  M5.Axp.SetLcdVoltage(3000);
  LogDebug("\n Starting");      // Start LogDebug 
  

  // xTaskCreatePinnedToCore(espNowRemoteTask,      /* Task function. */
  //                           "espNowRemoteTask",  /* name of task. */
  //                           4096,               /* Stack size of task */
  //                           NULL,               /* parameter of the task */
  //                           5,                  /* priority of the task */
  //                           &eRemote_t,         /* Task handle to keep track of created task */
  //                           0);                 /* pin task to core 0 */
  // delay(100);

  espNowRemoteTask();

  encoder1.attachHalfQuad(ENC_1_CLK, ENC_1_DT);
  encoder2.attachHalfQuad(ENC_2_CLK, ENC_2_DT);
  encoder3.attachHalfQuad(ENC_3_CLK, ENC_3_DT);
  encoder4.attachHalfQuad(ENC_4_CLK, ENC_4_DT);
  Button1.attachClick(mxclick);
  Button1.attachLongPressStop(mxlong);
  Button2.attachClick(click2);
  Button3.attachClick(click3);

  tft_lv_initialization();
  init_disp_driver();
  init_touch_driver();


  //****Load EEPROOM:
  eject_status = EEPROM.readBool(EJECT);
  dark_mode = EEPROM.readBool(DARKMODE);
  vibrate_mode = EEPROM.readBool(VIBRATE);
  touch_home = EEPROM.readBool(LEFTY);

  ui_init();
  
  if(eject_status == true){
  lv_obj_add_state(ui_ejectaddon, LV_STATE_CHECKED);
  lv_obj_clear_state(ui_EJECTSettingButton, LV_STATE_DISABLED);
  lv_obj_clear_state(ui_HomeButtonL, LV_STATE_DISABLED);
  }
  if(dark_mode == true){
  lv_obj_add_state(ui_darkmode, LV_STATE_CHECKED);
  }
  if(vibrate_mode == true){
  lv_obj_add_state(ui_vibrate, LV_STATE_CHECKED);
  }
  if(touch_home == true){
  lv_obj_add_state(ui_lefty, LV_STATE_CHECKED);
  }
  lv_roller_set_selected(ui_PatternS,2,LV_ANIM_OFF);
  lv_roller_get_selected_str(ui_PatternS,patternstr,0);
  lv_label_set_text(ui_HomePatternLabel,patternstr);
}

void loop()
{
     const int BatteryLevel = M5.Axp.GetBatteryLevel();
     String BatteryValue = (String(BatteryLevel, DEC) + "%");
     const char *battVal = BatteryValue.c_str();
     lv_bar_set_value(ui_Battery, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue, battVal);
     lv_bar_set_value(ui_Battery1, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue1, battVal);
     lv_bar_set_value(ui_Battery2, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue2, battVal);
     lv_bar_set_value(ui_Battery3, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue3, battVal);
     lv_bar_set_value(ui_Battery4, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue4, battVal);
     lv_bar_set_value(ui_Battery5, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue5, battVal);

     M5.update();
     lv_task_handler();
     Button1.tick();
     Button2.tick();
     Button3.tick();

     switch(st_screens){
      
     case ST_UI_START:
      {
        if(click2_short_waspressed == true){
         lv_event_send(ui_StartButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_StartButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_event_send(ui_StartButtonR, LV_EVENT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_HOME:
      {
        if(touch_home == true){
          touch_disabled = true;
        }
        // Encoder 1 Speed 
        float encoder_max = known_ossm.maxspeed;
        if(lv_slider_is_dragged(ui_homespeedslider) == false){
          if (encoder1.getCount() != speedenc){
            lv_slider_set_value(ui_homespeedslider, want_ossm_state.speed, LV_ANIM_OFF);

            int multiplier = (speedenc / 50);
            long diff = encoder1.getCount() - speedenc;
            encoder1.setCount(encoder1.getCount() + (diff * multiplier));

            if(encoder1.getCount() <= 0){
              encoder1.setCount(0);
            } else if (encoder1.getCount() >= encoder_max){
              encoder1.setCount(encoder_max);
            } 
            speedenc = encoder1.getCount();
            want_ossm_state.speed = fscale(0, encoder_max, 0, known_ossm.maxspeed, speedenc, 0);
            send_ossm_state();
          }
        } else if(lv_slider_get_value(ui_homespeedslider) != want_ossm_state.speed){
            speedenc =  fscale(0.5, known_ossm.maxspeed, 0, encoder_max, want_ossm_state.speed, speedscale);
            encoder1.setCount(speedenc);
            want_ossm_state.speed = lv_slider_get_value(ui_homespeedslider);
            send_ossm_state();
        }
        char speed_v[7];
        dtostrf(want_ossm_state.speed, 6, 0, speed_v);
        lv_label_set_text(ui_homespeedvalue, speed_v);


        // Encoder2 Depth
        encoder_max = known_ossm.maxdepth / 2;
        if(lv_slider_is_dragged(ui_homedepthslider) == false){
          if (encoder2.getCount() != depthenc){
            lv_slider_set_value(ui_homedepthslider, want_ossm_state.depth, LV_ANIM_OFF);
            if(encoder2.getCount() <= 0){
              encoder2.setCount(0);
            } else if (encoder2.getCount() >= encoder_max){
              encoder2.setCount(encoder_max);
            } 
            depthenc = encoder2.getCount();
            want_ossm_state.depth = fscale(0, encoder_max, 0, known_ossm.maxdepth, depthenc, 0);
            send_ossm_state();
          }
        } else if(lv_slider_get_value(ui_homedepthslider) != want_ossm_state.depth){
            depthenc =  fscale(0, known_ossm.maxdepth, 0, encoder_max, want_ossm_state.depth, 0);
            encoder2.setCount(depthenc);
            want_ossm_state.depth = lv_slider_get_value(ui_homedepthslider);
            send_ossm_state();
        }
        char depth_v[7];
        dtostrf(want_ossm_state.depth, 6, 0, depth_v);
        lv_label_set_text(ui_homedepthvalue, depth_v);
        

        // Encoder3 Stroke
        encoder_max = known_ossm.maxdepth / 2;
        if(lv_slider_is_dragged(ui_homestrokeslider) == false){
          if (encoder3.getCount() != strokeenc){
            lv_slider_set_value(ui_homestrokeslider, want_ossm_state.stroke, LV_ANIM_OFF);
            if(encoder3.getCount() <= 0){
              encoder3.setCount(0);
            } else if (encoder3.getCount() >= encoder_max){
              encoder3.setCount(encoder_max);
            } 
            strokeenc = encoder3.getCount();
            want_ossm_state.stroke = fscale(0, encoder_max, 0, known_ossm.maxdepth, strokeenc, 0);
            send_ossm_state();
          }
        } else if(lv_slider_get_value(ui_homestrokeslider) != want_ossm_state.stroke){
            strokeenc =  fscale(0, known_ossm.maxdepth, 0, encoder_max, want_ossm_state.stroke, 0);
            encoder3.setCount(strokeenc);
            want_ossm_state.stroke = lv_slider_get_value(ui_homestrokeslider);
            send_ossm_state();
        }
        char stroke_v[7];
        dtostrf(want_ossm_state.stroke, 6, 0, stroke_v);
        lv_label_set_text(ui_homestrokevalue, stroke_v);

        // Encoder4 Senstation
        if(lv_slider_is_dragged(ui_homesensationslider) == false){
          if (encoder4.getCount() != sensationenc){
            lv_slider_set_value(ui_homesensationslider, want_ossm_state.sensation, LV_ANIM_OFF);
            if(encoder4.getCount() <= (Encoder_MAP/2*-1)){
              encoder4.setCount((Encoder_MAP/2*-1));
            } else if (encoder4.getCount() >= (Encoder_MAP/2)){
              encoder4.setCount((Encoder_MAP/2));
            } 
            sensationenc = encoder4.getCount();
            want_ossm_state.sensation = fscale((Encoder_MAP/2*-1), (Encoder_MAP/2), -100, 100, sensationenc, 0);
            send_ossm_state();
          }
        } else if(lv_slider_get_value(ui_homesensationslider) != want_ossm_state.sensation){
            sensationenc =  fscale(-100, 100, (Encoder_MAP/2*-1), (Encoder_MAP/2), want_ossm_state.sensation, 0);
            encoder4.setCount(sensationenc);
            want_ossm_state.sensation = lv_slider_get_value(ui_homesensationslider);
            send_ossm_state();
        }

        if(click2_short_waspressed == true){
         lv_event_send(ui_HomeButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_HomeButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_event_send(ui_HomeButtonR, LV_EVENT_CLICKED, NULL);
        }
        

      }
      break;

      case ST_UI_MENUE:
      {
        if(touch_home == true){
          touch_disabled = true;
        }
        if(encoder4.getCount() > encoder4_enc + 2){
          LogDebug("next");
          lv_group_focus_next(ui_g_menue);
          encoder4_enc = encoder4.getCount();
        } else if(encoder4.getCount() < encoder4_enc -2){
          lv_group_focus_prev(ui_g_menue);
          LogDebug("Preview");
          encoder4_enc = encoder4.getCount();
        }

        if(click2_short_waspressed == true){
         lv_event_send(ui_MenueButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_MenueButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_event_send(lv_group_get_focused(ui_g_menue), LV_EVENT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_PATTERN:
      {
        if(touch_home == true){
          touch_disabled = true;
        }
        if(encoder4.getCount() > encoder4_enc + 2){
          LogDebug("next");
          uint32_t t = LV_KEY_DOWN;
          lv_event_send(ui_PatternS, LV_EVENT_KEY, &t);
          encoder4_enc = encoder4.getCount();
        } else if(encoder4.getCount() < encoder4_enc -2){
          uint32_t t = LV_KEY_UP;
          lv_event_send(ui_PatternS, LV_EVENT_KEY, &t);
          LogDebug("Preview");
          encoder4_enc = encoder4.getCount();
        }
         if(click2_short_waspressed == true){
         lv_event_send(ui_PatternButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_PatternButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_event_send(ui_PatternButtonR, LV_EVENT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_Torqe:
      {
        if(touch_home == true){
          touch_disabled = true;
        }
        // Encoder 1 Torqe Out
        if(lv_slider_is_dragged(ui_outtroqeslider) == false){
          if (encoder1.getCount() != torqe_f_enc){
            lv_slider_set_value(ui_outtroqeslider, want_ossm_state.torque_f, LV_ANIM_OFF);
            if(encoder1.getCount() <= 0){
              encoder1.setCount(0);
            } else if (encoder1.getCount() >= Encoder_MAP){
              encoder1.setCount(Encoder_MAP);
            } 
            torqe_f_enc = encoder1.getCount();
            want_ossm_state.torque_f = fscale(0, Encoder_MAP, 50, 200, torqe_f_enc, 0);
            send_ossm_state();
          }
        } else if(lv_slider_get_value(ui_outtroqeslider) != want_ossm_state.torque_f){
            torqe_f_enc = fscale(50, 200, 0, Encoder_MAP, want_ossm_state.torque_f, 0);
            encoder1.setCount(torqe_f_enc);
            want_ossm_state.torque_f = lv_slider_get_value(ui_outtroqeslider);
            send_ossm_state();
        }
        char torqe_f_v[7];
        dtostrf((want_ossm_state.torque_f*-1), 6, 0, torqe_f_v);
        lv_label_set_text(ui_outtroqevalue, torqe_f_v);

        // Encoder 4 Torqe IN
        if(lv_slider_is_dragged(ui_introqeslider) == false){
          if (encoder4.getCount() != torqe_r_enc){
            lv_slider_set_value(ui_introqeslider, want_ossm_state.torque_r, LV_ANIM_OFF);
            if(encoder4.getCount() <= 0){
              encoder4.setCount(0);
            } else if (encoder4.getCount() >= Encoder_MAP){
              encoder4.setCount(Encoder_MAP);
            } 
            torqe_r_enc = encoder4.getCount();
            want_ossm_state.torque_r = fscale(0, Encoder_MAP, 20, 200, torqe_r_enc, 0);
            send_ossm_state();
          }
        } else if(lv_slider_get_value(ui_introqeslider) != want_ossm_state.torque_r){
            torqe_r_enc = fscale(20, 200, 0, Encoder_MAP, want_ossm_state.torque_r, 0);
            encoder4.setCount(torqe_r_enc);
            want_ossm_state.torque_r = lv_slider_get_value(ui_introqeslider);
            send_ossm_state();
        }
        char torqe_r_v[7];
        dtostrf(want_ossm_state.torque_r, 6, 0, torqe_r_v);
        lv_label_set_text(ui_introqevalue, torqe_r_v);

         if(click2_short_waspressed == true){
         lv_event_send(ui_TorqeButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_TorqeButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_event_send(ui_TorqeButtonR, LV_EVENT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_EJECTSETTINGS:
      {
        if(touch_home == true){
          touch_disabled = true;
        }
        
         if(click2_short_waspressed == true){
         lv_event_send(ui_EJECTButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_EJECTButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         
        }
      }
      break;

      case ST_UI_SETTINGS:
      {
        touch_disabled = false;
        if(encoder4.getCount() > encoder4_enc + 2){
          LogDebug("next");
          lv_group_focus_next(ui_g_settings);
          encoder4_enc = encoder4.getCount();
        } else if(encoder4.getCount() < encoder4_enc -2){
          lv_group_focus_prev(ui_g_settings);
          LogDebug("Preview");
          encoder4_enc = encoder4.getCount();
        }

        if(click2_short_waspressed == true){
         lv_event_send(ui_MenueButtonL, LV_EVENT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_event_send(ui_MenueButtonM, LV_EVENT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_event_send(ui_EJECTButtonR, LV_EVENT_CLICKED, NULL);
        }
      }
      break;

     }
     mxclick_long_waspressed = false;
     mxclick_short_waspressed = false;
     click2_short_waspressed = false;
     click3_short_waspressed = false;

  int now = millis();
  if(ossm_connected && now - last_control_sent >= HEARTBEAT_INTERVAL) {
    last_control_sent = now;
    send_ossm_state();
  }
  // if(m5_first_connect && outgoingcontrol.esp_connected && now - lastControlSent >= 500) {
  //   switch(outgoingcontrol.esp_command) {
  //     case SPEED:
  //     case DEPTH:
  //     case STROKE:
  //     case SENSATION:
  //     case ON:
  //     case OFF:
  //       LogDebug("Resending command");
  //       esp_now_send(Broadcast_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
  //       lastControlSent = millis();
  //       break;
  //   }
  // }

  delay(5);
}

void espNowRemoteTask()
{
  WiFi.mode(WIFI_STA);
  LogDebug(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, Broadcast_Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // for(;;) {
  //   vTaskDelay(200);
  // }
}

/*
void cumscreentask(void *pvParameters)
{
  for(;;)
  {
    M5.Lcd.setTextColor(FrontColor);
    if(encoder1.getCount() != cum_s_enc)
    {
    cum_s_enc = encoder1.getCount();
    cum_speed = map(constrain(cum_s_enc,0,Encoder_MAP),0,Encoder_MAP,1000,30000);
    M5.Lcd.fillRect(199,S1Pos,85,30,BgColor);
    M5.Lcd.setCursor(200,S1Pos+progheight-5);
    M5.Lcd.print(cum_speed);
    SendCommand(CUMSPEED, cum_speed, CUM);
    }

  if(encoder2.getCount() != cum_t_enc)
    {
    cum_t_enc = encoder2.getCount();
    cum_time = map(constrain(cum_t_enc,0,Encoder_MAP),0,Encoder_MAP,0,60);
    M5.Lcd.fillRect(199,S2Pos,85,30,BgColor);
    M5.Lcd.setCursor(200,S2Pos+progheight-5);
    M5.Lcd.print(cum_time);
    SendCommand(CUMTIME, cum_time, CUM);
    }

   if(encoder3.getCount() != cum_si_enc)
    {
    cum_si_enc = encoder3.getCount();
    cum_size = map(constrain(cum_si_enc,0,Encoder_MAP),0,Encoder_MAP,0,40);
    M5.Lcd.fillRect(199,S3Pos,85,30,BgColor);
    M5.Lcd.setCursor(200,S3Pos+progheight-5);
    M5.Lcd.print(cum_size);
    SendCommand(CUMSIZE, cum_size, CUM);
    }

   if(encoder4.getCount() != cum_a_enc)
    {
    cum_a_enc = encoder4.getCount();
    cum_accel = map(constrain(cum_a_enc,0,Encoder_MAP),0,Encoder_MAP,0,20);
    M5.Lcd.fillRect(199,S4Pos,85,30,BgColor);
    M5.Lcd.setCursor(200,S4Pos+progheight-5);
    M5.Lcd.print(cum_accel);
    SendCommand(CUMACCEL, cum_accel, CUM);
    }
  vTaskDelay(100);
  }
}
*/

void vibrate(){
    if(vibrate_mode == true){
    M5.Axp.SetLDOEnable(3,true);
    vTaskDelay(300);
    M5.Axp.SetLDOEnable(3,false);
    }
}

void mxclick() {
  vibrate();
  mxclick_short_waspressed = true;
} 

void mxlong(){
  vibrate();
  mxclick_long_waspressed = true;
} 

void click2() {
  vibrate();
  click2_short_waspressed = true;
} // click1

void click3() {
  vibrate();
  click3_short_waspressed = true;
} // click1