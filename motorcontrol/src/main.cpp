#include <Arduino.h>
#include <WiFi.h>
#include "driver/pcnt.h"


//define WLAN
#define WLAN_SSID "SPL_B"
#define WLAN_PASS "Nao?!Nao?!"
#define LOCAL_IP {10,0,13,170} //{10, 0, 13, 170}
#define GATEWAY {10, 0, 13, 1}
#define SUBNET {255, 255, 255, 0}

//define behaviour
#define POS_NO_MOVEMENT_THRESHOLD 10
#define NEG_NO_MOVEMENT_THRESHOLD -10

//define the number of motors and the pins for the sensors
#define MOTORS 1
const int SENSOR_PINS[] = {32, 35};
const pcnt_unit_t PCNT_UNIT[] = {PCNT_UNIT_0};

//typecode 0x01
struct Status_struct {
  u16_t magicNumber;
};

//typecode 0x02
struct Status_Movement_struct {
  int16_t extrute[MOTORS];
};  

struct Status_return_struct {
  u16_t magicNumber;
  byte sensor[MOTORS*2];
  int16_t counter[MOTORS];
  int16_t target[MOTORS];
};

char* Status_return_struct_to_string(Status_return_struct data){
  char* buffer = (char*)malloc(100);
  sprintf(buffer, "magicNumber: %i, sensor: %i %i, counter: %i, target: %i", data.magicNumber, data.sensor[0], data.sensor[1], data.counter[0], data.target[0]);
  return buffer;
}

WiFiServer server(80);
int16_t target[MOTORS];

Status_return_struct pocess_status_struct(Status_struct data){
  Serial.println("Processing Status");
  Status_return_struct return_data;
  return_data.magicNumber = data.magicNumber;
  for (int i = 0; i < MOTORS; i++) {
    return_data.sensor[i*2] = digitalRead(SENSOR_PINS[i*2]);
    return_data.sensor[i*2+1] = digitalRead(SENSOR_PINS[i*2+1]);
    int16_t counter;
    pcnt_get_counter_value(PCNT_UNIT[i], &counter);
    return_data.counter[i] = counter;
    return_data.target[i] = target[i];
  }
  return return_data;
}

void process_movement_struct(Status_Movement_struct data){
  for (int i = 0; i < MOTORS; i++) {
    target[i] += data.extrute[i];
  }
  Serial.println("Processing Movement");
}

bool wireless_connection_setup() {
  // Connect to the wireless network
  const char* ssid = WLAN_SSID;
  const char* password = WLAN_PASS;

  IPAddress local_IP(LOCAL_IP);
  IPAddress gateway(GATEWAY);  
  IPAddress subnet(SUBNET);  
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
    return false;
  }
  WiFi.begin(ssid, password);
  for (int i = 0; i < 10 ; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to the WiFi network");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    Serial.println("Connecting to WiFi...");
    delay(1000);
  }
  return false;
}

void counter_setup(){
  for (int i = 0; i < MOTORS; i++) {
    //gpio_num_t pin_a = (gpio_num_t) SENSOR_PINS[i*2];
    //gpio_num_t pin_b = (gpio_num_t) SENSOR_PINS[i*2+1];
  	//gpio_pad_select_gpio(pin_a);
  	//gpio_pad_select_gpio(pin_b);
	  //gpio_set_direction(pin_a, GPIO_MODE_INPUT);
	  //gpio_set_direction(pin_b, GPIO_MODE_INPUT);

    pcnt_config_t pcnt_config;  
    pcnt_config.pulse_gpio_num = SENSOR_PINS[i*2];
    pcnt_config.ctrl_gpio_num = SENSOR_PINS[i*2+1];
    
    
    pcnt_config.unit = PCNT_UNIT[i];
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
    
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config);

    pcnt_config_t pcnt_config2;

    pcnt_config2.pulse_gpio_num = SENSOR_PINS[i*2+1];
    pcnt_config2.ctrl_gpio_num = SENSOR_PINS[i*2];
    
    pcnt_config2.unit = PCNT_UNIT[i];
    pcnt_config2.channel = PCNT_CHANNEL_1;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    
    pcnt_config2.counter_h_lim = 32767;
    pcnt_config2.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config2);
  
    //new since 28.08.2024
    pcnt_counter_pause(PCNT_UNIT[i]);
    pcnt_counter_clear(PCNT_UNIT[i]);
    //pcnt_intr_enable(PCNT_UNIT[i]);
    pcnt_counter_resume(PCNT_UNIT[i]);
  

    Serial.println("Configured pcnt");
  }
}

void set_motor_speed(int motor, bool direction, int16_t speed){
  //Status_struct data;
  //data.magicNumber = 0x01;
  //Status_return_struct return_data = pocess_status_struct(data);
  //Serial.println(Status_return_struct_to_string(return_data));
  if(speed == 0){
    return;
  }
  digitalWrite(22, direction);
  digitalWrite(23, HIGH);
  delay(1);
  digitalWrite(23, LOW);  
  //Serial.println("Motor Speed Set");
}

void motor_control_loop(void* pvParameters) {
  while (true) {
    vTaskDelay(1);
    for (int i = 0; i < MOTORS; i++) {
      int16_t counter;
      pcnt_get_counter_value(PCNT_UNIT[i], &counter); 
      int16_t movement = target[i] - counter;
      //stop if motor is beetwen the thresholds
      if (movement <= POS_NO_MOVEMENT_THRESHOLD && movement >= NEG_NO_MOVEMENT_THRESHOLD) {
        set_motor_speed(i, true, 0);
        continue;
      }    
      set_motor_speed(i,movement < 0, movement); 
    }
  }
}

void setup() {
  Serial.begin(115200);
  bool is_connected = wireless_connection_setup();
  if (!is_connected) {
    return;
  }
  for (int i = 0; i < MOTORS; i++) {
    target[i] = 0; 
  }
  counter_setup();

  //start the motor control loop in a separate thread
  
  xTaskCreatePinnedToCore(
    motor_control_loop,   /* Function to implement the task */
    "motor_control_loop", /* Name of the task */
    2048,      /* Stack size in words */
    NULL,       /* Task input parameter */
    1,          /* Priority of the task */
    NULL ,      /* Task handle. */
    1
  );         /* Core where the task should run */
  server.begin();

  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

}

void server_loop(){
  WiFiClient client = server.available();
  if(client){
    Serial.println("new client connected");
    while (client.connected()){
      if (client.available()) {
        uint8_t type_code;
        client.read(&type_code, sizeof(type_code));
        Serial.println(type_code);
        switch (type_code) {
          case 0x01: {
            Status_struct data;
            client.read((uint8_t*)&data, sizeof(data));
            Status_return_struct return_data = pocess_status_struct(data);
            Serial.println("Sending Status");
            Serial.println(Status_return_struct_to_string(return_data));
            client.write((uint8_t*)&return_data, sizeof(return_data));
            break;
          }
          case 0x02: {
            Status_Movement_struct status;
            client.read((uint8_t*)&status, sizeof(status));
            process_movement_struct(status);
            break;
          }
        }
        break;
      }
      delay(10);
    }
    delay(10);
    client.stop();
    Serial.println("client disconnected");
  }
}

void loop() {
  server_loop();
}