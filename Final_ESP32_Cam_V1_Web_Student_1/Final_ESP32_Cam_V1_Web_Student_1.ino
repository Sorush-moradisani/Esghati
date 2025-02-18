#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "fb_gfx.h"

const char* ssid = "your ssid";
const char* password = "your password";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
#define LED_BUILTIN 4
bool INT_LED = false;
hw_timer_t *My_timer = NULL;
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

#define RED 15 // pin 12 can also be used
#define GREEN 14
#define BLUE 2
#define Mic 12
int light = 0;
unsigned long door_opened_millis = 0;
long interval = 5000;           // open lock for ... milliseconds
bool face_recognised = false;

const int numReadings = 5;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;
int total = 0;                  // the running total
int average_face_size = 0;      // the average
int face_distance;
int face_id = 0;
String command;
bool commandEnd=false;
char inByte;
int arg = 0;
bool follow = 0;

//void app_facenet_main();
//void app_httpserver_init();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;


static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;
void IRAM_ATTR onTimer(){
  Serial.println("task");
}
void setup() {
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode (LED_BUILTIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(33, OUTPUT); 
  pinMode(Mic, INPUT);
  digitalWrite(33, HIGH);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("ACCESS POINT IP address: ");
  Serial.println(IP);

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 300000000, true);
  timerAlarmEnable(My_timer);
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); //send face to browser
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  timerWrite(My_timer, 0); //reset timer (feed watchdog)
  //Serial.println(msg.data());
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }else if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }else if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }else if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }else if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }else if (msg.data().substring(0, 5) == "tilt:") {
    //int i = msg.data().substring(5).toInt();
    Serial.print("tilt=");
    Serial.println(msg.data().substring(5));
  }else if (msg.data().substring(0, 4) == "pan:") {
    //int j = msg.data().substring(4).toInt();
    Serial.print("pan=");
    Serial.println(msg.data().substring(4));
  }else if (msg.data().substring(0, 9) == "lr-rhand:") {
    //int i = msg.data().substring(5).toInt();
    Serial.print("lr-rhand=");
    Serial.println(msg.data().substring(9));
  }else if (msg.data().substring(0, 9) == "ud-rhand:") {
    //int j = msg.data().substring(4).toInt();
    Serial.print("ud-rhand=");
    Serial.println(msg.data().substring(9));
  }else if (msg.data().substring(0, 9) == "lr-lhand:") {
    //int i = msg.data().substring(5).toInt();
    Serial.print("lr-lhand=");
    Serial.println(msg.data().substring(9));
  }else if (msg.data().substring(0, 9) == "ud-lhand:") {
    //int j = msg.data().substring(4).toInt();
    Serial.print("ud-lhand=");
    Serial.println(msg.data().substring(9));
  }else if (msg.data() == "stop") {
    Serial.println("stop");
  }else if (msg.data() == "r-rotate") {
    Serial.println("r-rotate");
  }else if (msg.data() == "l-rotate") {
    Serial.println("l-rotate");
  }else if (msg.data() == "f-left") {
    Serial.println("f-left");
  }else if (msg.data() == "f-forward") {
    Serial.println("f-forward");
  }else if (msg.data() == "f-right") {
    Serial.println("f-right");
  }else if (msg.data() == "b-left") {
    Serial.println("b-left");
  }else if (msg.data() == "b-backward") {
    Serial.println("b-backward");
  }else if (msg.data() == "b-right") {
    Serial.println("b-right");
  }else if (msg.data() == "introduce") {
    Serial.println("introduce");
  }else if (msg.data() == "greet") {
    Serial.println("greet");
  }else if (msg.data() == "find" && g_state == START_RECOGNITION) {
    Serial.println("find");
  }else if (msg.data() == "follow") {
    
    if(!follow){
        Serial.println("follow");
        follow = true;
    }else{
      Serial.println("nofollow");
      follow = false;
    }
    
  }else if (msg.data() == "dance") {
    Serial.println("dance");
  }else if (msg.data() == "random") {
    Serial.println("random");
  }else if (msg.data() == "history") {
    Serial.println("history");
  }else if (msg.data() == "hymn") {
    Serial.println("hymn");
  }else if (msg.data() == "hreset") {
    Serial.println("hreset");
  }else if (msg.data() == "reset") {
    Serial.println("reset");
  }else if (msg.data() == "light") {
    if(INT_LED){
      digitalWrite(LED_BUILTIN, LOW);
      INT_LED = false;
    }else{
      digitalWrite(LED_BUILTIN, HIGH);
      INT_LED = true;
    }
    
    Serial.println("light");
  }else if (msg.data() == "rgb") {
    if(light == 0){
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);
      light++;
      }else if(light == 1){
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(BLUE, LOW);
      light++;
      }else if(light == 2){
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
      light++;
      }else{
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);
      light=0;
      }
    Serial.println("rgb");
  }else if (msg.data() == "fire") {
    Serial.println("fire");
  }else if (msg.data() == "voicemode") {
    Serial.println("voicemode");
  }else if (msg.data() == "clearcache") {
    Serial.println("free heap: ");
    Serial.println(ESP.getFreeHeap());
  }else if (msg.data() == "seq1") {
    Serial.println("seq1");
  }else if (msg.data() == "seq2") {
    Serial.println("seq2");
  }else if (msg.data() == "seq3") {
    Serial.println("seq3");
  }else if (msg.data() == "seq4") {
    Serial.println("seq4");
  }
  
}

void open_door(WebsocketsClient &client) {
  if (true/*digitalRead(relay_pin) == LOW*/) {
    //digitalWrite(relay_pin, HIGH); //close (energise) relay so door unlocks
    Serial.println("sorush");
    client.send("door_open");
    door_opened_millis = millis(); // time relay closed and door opened
  }
}
static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes)
{
  int x, y, w, h, i, half_width, half_height;
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {

    // Convoluted way of finding face centre...
    x = ((int)boxes->box[i].box_p[0]);
    w = (int)boxes->box[i].box_p[2] - x + 1;
    half_width = w / 2;
    int face_center_pan = x + half_width; // current face centre x co-ordinate

    y = (int)boxes->box[i].box_p[1];
    h = (int)boxes->box[i].box_p[3] - y + 1;
    half_height = h / 2;
    int face_center_tilt = y + half_height;  // current face centre y co-ordinate

    // subtract the last reading:
    total = total - readings[readIndex];
    // add current face height:
    readings[readIndex] = h;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average_face_size = total / numReadings;

    int eq_top = 3.6 * 200 * 240; //f(mm) x real height(mm) x image height(px)
    int eq_bottom = average_face_size * 2.7; //object height(px) x sensor height(mm)
    int face_distance = eq_top / eq_bottom;

    /*Serial.print('<'); // start marker
    Serial.print(face_center_pan);
    Serial.print(','); // comma separator
    Serial.print(face_center_tilt);
    Serial.print(','); // comma separator
    Serial.print(face_distance);
    Serial.println('>'); // end marker*/
    Serial.print("autopan=");
    Serial.println(face_center_pan);
    Serial.print("autotilt=");
    Serial.println(face_center_tilt);
    Serial.print("distance=");
    Serial.println(face_distance);

  }
}
void loop() {
  
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;

  send_face_list(client);
  client.send("STREAMING");


  while (client.available()) {
    client.poll();

    // char voice[12];
    //   sprintf(voice, "voice:%s", (const char *)analogRead(12));
    //   client.send(voice);
    //   Serial.println(analogRead(12));
    //   client.sendBinary((const char *)analogRead(12));

    if (Serial.available()) {
      
    String command = Serial.readStringUntil('\n');
    if (command == "red") {
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);
    } else if (command == "green") {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(BLUE, LOW);
    } else if (command == "blue") {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
    } else if (command == "norgb") {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);
    }
  }

    if (millis() - interval > door_opened_millis) { // current time - face recognised time > 5 secs
      //digitalWrite(relay_pin, LOW); //open relay
    }

    fb = esp_camera_fb_get();

    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes)
      {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT) {
            client.send("FACE DETECTED");
          }
          if(follow){
            draw_face_boxes(image_matrix, out_res.net_boxes);
        }
          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);

            }
          }

          if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "Hi %s", f->id_name);
              open_door(client);
              client.send(recognised_message);
            }
            else
            {
              client.send("FACE NOT RECOGNISED");
              Serial.println("stranger");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }
          free(out_res.net_boxes->score);  // Free allocated memory
          free(out_res.net_boxes->box);
          free(out_res.net_boxes->landmark);
          free(out_res.net_boxes);
      }
      else
      {
        if (g_state != START_DETECT) {
          client.send("NO FACE DETECTED");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
        client.send("DETECTING");
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
}
boolean compareString(String a, String b) {
  if (a.length() != b.length() + 1) {
    return false;
  }
  for (int i = 0; i < a.length() - 1; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}