#include <lvgl.h>
#include <Wire.h>
#include <SPI.h>
#include "driver/twai.h"
#include "ui.h"
#include <stdio.h>
#include "gfx_conf.h"
#include <esp_now.h> // Add ESP-NOW library
#include <WiFi.h>    // Required for ESP-NOW

// Define message frequencies for different CAN IDs
const struct {
    uint32_t id;
    uint8_t freq_hz;
} CAN_MSG_FREQUENCIES[] = {
    {0x520, 50},  // RPM, TPS, MAP, Lambda - 50Hz
    {0x522, 50},  // Speed, Fuel duty - 50Hz
    {0x528, 10},  // Knock count - 10Hz
    {0x530, 10},  // Battery, IAT, Coolant - 10Hz
    {0x531, 10},  // Fuel trim, Ignition - 10Hz
    //{0x534, 10},  // Error codes, Lost sync - 10Hz
    {0x537, 10},   // Fuel pressure - 10Hz
    {0x538, 10}    //Rotrex Ratio and speed - 10Hz
};

struct ProcessedDataPacket {
    int16_t rpm;
    int16_t map;
    int16_t lambda;
    int16_t speed;
    int16_t fuelduty;
    int16_t battery_voltage;
    int16_t intake_air_temp;
    int16_t coolant_temp;
    int16_t knock_count;
    int16_t fuel_pressure;
    int16_t ratio;
    int16_t turbo_speed;
    int16_t total_ignition_comp;
    float boost_psi;
    uint32_t timestamp;
};

uint8_t receiverMacAddress[] = {0xCC, 0xBA, 0x97, 0x10, 0x4D, 0x90}; // Replace with actual MAC
esp_now_peer_info_t peerInfo;
bool espNowInitialized = false;

// Structure for value buffering
struct ValueBuffer {
    int16_t current_value;
    int16_t last_value;
    uint32_t last_update_time;
    bool has_new_data;
};

// Structure for all display values
struct DisplayValues {
    ValueBuffer rpm;
    //ValueBuffer throttle_position;
    ValueBuffer map;
    ValueBuffer lambda;
    ValueBuffer speed;
    ValueBuffer fuelduty;
    ValueBuffer battery_voltage;
    ValueBuffer intake_air_temp;
    ValueBuffer coolant_temp;
    ValueBuffer knock_count;
    ValueBuffer fuel_pressure;
    ValueBuffer ratio;
    ValueBuffer turbo_speed;
    //ValueBuffer error_code_count;
    //ValueBuffer lost_sync_count;
    //ValueBuffer total_fuel_trim;
    ValueBuffer total_ignition_comp;
} display_values = {0};

// Original CAN data structure
struct struct_message {
    //int16_t rpm;
    //int16_t throttle_position;
    //int16_t map;
    //int16_t lambda;
    //int16_t speed;
    //int16_t fuelduty;
    //int16_t battery_voltage;
    //int16_t intake_air_temp;
    //int16_t coolant_temp;
    //int16_t knock_count;
    //int16_t fuel_pressure;
    //int16_t ratio;
    //int16_t turbo_speed;
    //int16_t error_code_count;
    //int16_t lost_sync_count;
    //int16_t total_fuel_trim;
    //int16_t total_ignition_comp;
    uint32_t last_update_time[7];
} myData = {0};

const float KPA_TO_BAR = 0.01;  // Conversion factor from kPa to BAR
const float ATMOSPHERIC_BAR = 1.013; // Standard atmospheric pressure in BAR
const float KPA_TO_PSI = 0.145038;  // Conversion factor from kPa to PSI
const float ATMOSPHERIC_KPA = 101.325; // Standard atmospheric pressure in kPa

// LVGL display buffer configuration
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf1[screenWidth * screenHeight / 10];
static lv_color_t disp_draw_buf2[screenWidth * screenHeight / 10];
static lv_disp_drv_t disp_drv;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)&color_p->full);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touchX, touchY;
    bool touched = tft.getTouch(&touchX, &touchY);
    
    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

void updateValue(ValueBuffer& buffer, int16_t new_value) {
    buffer.last_value = buffer.current_value;
    buffer.current_value = new_value;
    buffer.last_update_time = millis();
    buffer.has_new_data = true;
}

void InitCanDriver() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_19, GPIO_NUM_20, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("Failed to install CAN driver");
        return;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("Failed to start CAN driver");
        return;
    }

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
        Serial.println("Failed to reconfigure alerts");
        return;
    }

    Serial.println("CAN driver initialized successfully");
}

void ProcessCanMessage(const twai_message_t& message) {
    uint32_t current_time = millis();
    int msg_index = -1;

    for (size_t i = 0; i < sizeof(CAN_MSG_FREQUENCIES) / sizeof(CAN_MSG_FREQUENCIES[0]); i++) {
        if (CAN_MSG_FREQUENCIES[i].id == message.identifier) {
            msg_index = i;
            break;
        }
    }

    if (msg_index == -1) return;

    uint32_t min_interval = 1000 / CAN_MSG_FREQUENCIES[msg_index].freq_hz;
    if (current_time - myData.last_update_time[msg_index] < min_interval) {
        return;
    }

    myData.last_update_time[msg_index] = current_time;

    switch (message.identifier) {
        case 0x520: {
            updateValue(display_values.rpm, (message.data[0] | (message.data[1] << 8)));
            //updateValue(display_values.throttle_position, (message.data[2] | (message.data[3] << 8)));
            updateValue(display_values.map, (message.data[4] | (message.data[5] << 8)));
            updateValue(display_values.lambda, (message.data[6] | (message.data[7] << 8)));
            break;
        }
        case 0x522: {
            updateValue(display_values.fuelduty, (message.data[2] | (message.data[3] << 8)));
            updateValue(display_values.speed, (message.data[6] | (message.data[7] << 8)));
            break;
        }
        case 0x528: {
            updateValue(display_values.knock_count, (message.data[4] | (message.data[5] << 8)));
            break;
        }
        case 0x530: {
            updateValue(display_values.battery_voltage, (message.data[0] | (message.data[1] << 8)));
            updateValue(display_values.intake_air_temp, (message.data[4] | (message.data[5] << 8)));
            updateValue(display_values.coolant_temp, (message.data[6] | (message.data[7] << 8)));
            break;
        }
        case 0x531: {
            //updateValue(display_values.total_fuel_trim, (message.data[0] | (message.data[1] << 8)));
            updateValue(display_values.total_ignition_comp, (message.data[4] | (message.data[5] << 8)));
            break;
        }
        case 0x600: {
            //updateValue(display_values.total_fuel_trim, (message.data[0] | (message.data[1] << 8)));
            updateValue(display_values.total_ignition_comp, (message.data[0] | (message.data[1] << 8)));
            break;
        }
        //case 0x534: {
            //updateValue(display_values.error_code_count, (message.data[4] | (message.data[5] << 8)));
            //updateValue(display_values.lost_sync_count, (message.data[6] | (message.data[7] << 8)));
        //    break;
        //}
        //case 0x537: {
            //updateValue(display_values.fuel_pressure, (message.data[0] | (message.data[1] << 8)));
        //    break;
        //}
        case 0x538: {
             updateValue(display_values.ratio, (message.data[0] | (message.data[1] << 8)));
             updateValue(display_values.turbo_speed, (message.data[2] | (message.data[3] << 8)));
             updateValue(display_values.fuel_pressure, (message.data[4] | (message.data[5] << 8)));
             break;
        }
    }
}

void updateBoostBarLabel(lv_obj_t* uiValue, float boostBar) {
    char buffer[10];
    
    // Format boost value with sign and two decimal places
    snprintf(buffer, sizeof(buffer), "%+.2f", boostBar);
    
    // Set color based on boost value
    uint32_t color;
    if (boostBar > 1.0) {  // High boost warning (> 1 bar boost)
        color = 0xEB4034;  // Red
    } else if (boostBar > 0) {  // Positive boost
        color = 0x06F70D;  // Green
    } else {  // Vacuum
        color = 0xFDFCFC;  // White
    }
    
    lv_obj_set_style_text_color(uiValue, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(uiValue, buffer);
}

void updateBoostLabel(lv_obj_t* uiValue, float boostPsi) {
    char buffer[10];
    
    if(boostPsi <= 0){
      return;
    }

    // Format boost value with sign and one decimal place
    snprintf(buffer, sizeof(buffer), "%+.1f", boostPsi);
    
    // Set color based on boost value
    uint32_t color;
    if (boostPsi > 15.0) {  // High boost warning
        color = 0xEB4034;  // Red
    } else if (boostPsi > 0) {  // Positive boost
        color = 0x06F70D;  // Green
    } else {  // Vacuum
        color = 0xFDFCFC;  // White
    }
    
    lv_obj_set_style_text_color(uiValue, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(uiValue, buffer);
}

void updateLabelScaledInt(lv_obj_t * uiValue, int16_t inputValue, int min, int max, int redline, uint32_t redlinecolor, uint32_t lowlinecolor, float scale) {
    if (uiValue == NULL) {
        Serial.println("Error: uiValue is NULL.");
        return;
    }

    inputValue = constrain(inputValue, min, max);
    float displayValue = inputValue * scale;

    int decimalPlaces = 0;
    float tempScale = scale;
    while (tempScale < 1.0 && decimalPlaces < 6) {
        tempScale *= 10;
        decimalPlaces++;
    }

   char tempBuffer[20];
    snprintf(tempBuffer, sizeof(tempBuffer), "%.*f", decimalPlaces, displayValue);

    // Remove trailing zeros and decimal point if necessary
    char* decimal = strchr(tempBuffer, '.');
    if (decimal != NULL) {
        char* end = tempBuffer + strlen(tempBuffer) - 1;
        while (end > decimal && *end == '0') {
            *end = '\0';
            end--;
        }
        if (end == decimal) {
            *end = '\0';  // Remove decimal point if no decimals
        }
    }

    if (inputValue > redline) {
        lv_obj_set_style_text_color(uiValue, lv_color_hex(redlinecolor), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        lv_obj_set_style_text_color(uiValue, lv_color_hex(lowlinecolor), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    lv_label_set_text(uiValue, tempBuffer);
}

void updateDisplayValue(lv_obj_t* uiValue, ValueBuffer& buffer, int min, int max, int redline, 
                       uint32_t redlinecolor, uint32_t lowlinecolor, float scale) {
    if (!buffer.has_new_data) {
        return;
    }

    if(buffer.current_value <= 0){
      return;
    }

    if (buffer.current_value != buffer.last_value) {
        updateLabelScaledInt(uiValue, buffer.current_value, min, max, redline, redlinecolor, lowlinecolor, scale);
    }
}

void updateAllDisplayValues() {
    // Update high-frequency values (50Hz)
    updateDisplayValue(ui_RPMValue, display_values.rpm, 0, 6500, 6000, 0xEB4034, 0xFDFCFC, 1);
    if (display_values.rpm.has_new_data) {
      if(display_values.rpm.current_value > 0){
         lv_bar_set_value(ui_RpmBar, display_values.rpm.current_value, LV_ANIM_ON);
      }
    }
    
    updateDisplayValue(ui_MapValue, display_values.map, 0, 2000, 1800, 0xEB4034, 0xFDFCFC, 0.1);
    updateDisplayValue(ui_AFRValue, display_values.lambda, 0, 2000, 2500, 0xEB4034, 0xFDFCFC, 0.001);
    //updateDisplayValue(ui_TPSValue, display_values.throttle_position, 0, 1000, 1000, 0xEB4034, 0xFDFCFC, 0.1);
    updateDisplayValue(ui_SpeedValue, display_values.speed, 0, 3000, 2750, 0xEB4034, 0xFDFCFC, 0.1);
    updateDisplayValue(ui_FueldutyValue, display_values.fuelduty, 0, 1000, 950, 0xEB4034, 0x06F70D, 0.1);

    // Update lower-frequency values (10Hz)
    updateDisplayValue(ui_KnockValue, display_values.knock_count, 0, 100, 50, 0xEB4034, 0x06F70D, 1);
    updateDisplayValue(ui_BatteryvoltageValue, display_values.battery_voltage, 0, 1600, 1500, 0xEB4034, 0xFDFCFC, 0.01);
    updateDisplayValue(ui_IATValue, display_values.intake_air_temp, 0, 1000, 500, 0xEB4034, 0xFDFCFC, 0.1);
    updateDisplayValue(ui_CoolantValue, display_values.coolant_temp, 0, 1200, 950, 0xEB4034, 0x06F70D, 0.1);
    //updateDisplayValue(ui_FueltrimValue, display_values.total_fuel_trim, 0, 1000, 1000, 0xEB4034, 0x06F70D, 0.1);
    updateDisplayValue(ui_IgnitionValue, display_values.total_ignition_comp, 0, 1000, 1000, 0xEB4034, 0x06F70D, 0.1);
    //updateDisplayValue(ui_ErrorcodeValue, display_values.error_code_count, 0, 100, 10, 0xEB4034, 0x06F70D, 1);
    //updateDisplayValue(ui_LostsyncValue, display_values.lost_sync_count, 0, 100, 30, 0xEB4034, 0x06F70D, 1);
    updateDisplayValue(ui_FuelPressureValue, display_values.fuel_pressure, 0, 4000, 3600, 0xEB4034, 0xFDFCFC, 0.1);
    updateDisplayValue(ui_RatioValue, display_values.ratio, 0, 300, 170, 0xEB4034, 0xFDFCFC, 0.01);
    updateDisplayValue(ui_TurboSpeedValue, display_values.turbo_speed, 0, 12000, 11500, 0xEB4034, 0xFDFCFC, 10);

    //float boostBar = mapToBar(display_values.map.current_value);
    //updateBoostBarLabel(ui_BoostValue, boostBar);
    float boostPsi = mapToPsi(display_values.map.current_value);
    updateBoostLabel(ui_BoostValue, boostPsi);
     if (display_values.map.has_new_data) {
        if(display_values.map.current_value > 0){
           lv_bar_set_value(ui_BoostBar, boostPsi, LV_ANIM_ON);
        }  
    }

    TransmitProcessedDataViaEspNow();

    // Reset has_new_data flags after update
    for (size_t i = 0; i < sizeof(display_values) / sizeof(ValueBuffer); i++) {
        ((ValueBuffer*)&display_values)[i].has_new_data = false;
    }
}

void GetCanMessages() {
    twai_message_t message;
    int messages_processed = 0;
    const int MAX_MESSAGES_PER_CYCLE = 10;

    while (messages_processed < MAX_MESSAGES_PER_CYCLE) {
        esp_err_t result = twai_receive(&message, 0);  // Non-blocking receive
        
        if (result == ESP_OK) {
            ProcessCanMessage(message);
            messages_processed++;
        } else if (result == ESP_ERR_TIMEOUT) {
            break;
        } else {
            Serial.println("Error receiving CAN message");
            break;
        }
    }
}

float mapToBar(int16_t mapValue) {
    // Convert raw MAP value to kPa (assuming ECU sends MAP in kPa * 10)
    float mapKpa = mapValue * 0.1;
    
    // Convert kPa to BAR
    float mapBar = mapKpa * KPA_TO_BAR;
    
    // Calculate boost by subtracting atmospheric pressure
    float boostBar = mapBar - ATMOSPHERIC_BAR;
    
    // Round to 2 decimal places
    return roundf(boostBar * 100) / 100;
}

float mapToPsi(int16_t mapValue) {
    // Convert raw MAP value to kPa (assuming ECU sends MAP in kPa * 10)
    float mapKpa = mapValue * 0.1;
    
    // Calculate boost by subtracting atmospheric pressure and converting to PSI
    float boostPsi = (mapKpa - ATMOSPHERIC_KPA) * KPA_TO_PSI;
    
    // Round to 1 decimal place
    return roundf(boostPsi * 10) / 10;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW send failed");
    }
}

void InitEspNow() {
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register the send callback
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    
    espNowInitialized = true;
    Serial.println("ESP-NOW initialized successfully");
    
    // Print this device's MAC address for pairing purposes
    Serial.print("This device's MAC Address: ");
    Serial.println(WiFi.macAddress());
}

void TransmitProcessedDataViaEspNow() {
    if (!espNowInitialized) {
        return;
    }
    
    // Only send if we have new data to send
    bool has_new_data = false;
    for (size_t i = 0; i < sizeof(display_values) / sizeof(ValueBuffer); i++) {
        if (((ValueBuffer*)&display_values)[i].has_new_data) {
            has_new_data = true;
            break;
        }
    }
    
    if (!has_new_data) {
        return;
    }

    
    
    ProcessedDataPacket dataPacket;
    
    // Fill the packet with current values
    dataPacket.rpm = display_values.rpm.current_value;
    dataPacket.map = display_values.map.current_value;
    dataPacket.lambda = display_values.lambda.current_value;
    dataPacket.speed = display_values.speed.current_value;
    dataPacket.fuelduty = display_values.fuelduty.current_value;
    dataPacket.battery_voltage = display_values.battery_voltage.current_value;
    dataPacket.intake_air_temp = display_values.intake_air_temp.current_value;
    dataPacket.coolant_temp = display_values.coolant_temp.current_value;
    dataPacket.knock_count = display_values.knock_count.current_value;
    dataPacket.fuel_pressure = display_values.fuel_pressure.current_value;
    dataPacket.ratio = display_values.ratio.current_value;
    dataPacket.turbo_speed = display_values.turbo_speed.current_value;
    dataPacket.total_ignition_comp = display_values.total_ignition_comp.current_value;
    dataPacket.boost_psi = mapToPsi(display_values.map.current_value);
    dataPacket.timestamp = millis();
    
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &dataPacket, sizeof(ProcessedDataPacket));
    
    if (result != ESP_OK) {
        Serial.println("Error sending processed data via ESP-NOW");
    }
}


void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    // GPIO initialization based on panel type
    #if defined (CrowPanel_50) || defined (CrowPanel_70)
        pinMode(38, OUTPUT);
        digitalWrite(38, LOW);
        pinMode(17, OUTPUT);
        digitalWrite(17, LOW);
        pinMode(18, OUTPUT);
        digitalWrite(18, LOW);
        pinMode(42, OUTPUT);
        digitalWrite(42, LOW);
    #elif defined (CrowPanel_43)
        pinMode(20, OUTPUT);
        digitalWrite(20, LOW);
        pinMode(19, OUTPUT);
        digitalWrite(19, LOW);
        pinMode(35, OUTPUT);
        digitalWrite(35, LOW);
        pinMode(38, OUTPUT);
        digitalWrite(38, LOW);
        pinMode(0, OUTPUT);//TOUCH-CS
    #endif

    // Display initialization
    tft.begin();
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    delay(200);

    // Initialize LVGL
    lv_init();
    delay(100);

    // Initialize display buffer
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, screenWidth * screenHeight/10);

    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.full_refresh = 1;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize touchpad
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Clear screen
    tft.fillScreen(TFT_BLACK);

    // Initialize UI
    ui_init();

    // Initialize CAN driver
    InitCanDriver();

     // Initialize ESP-NOW
    InitEspNow();

    // Initialize all display values to zero
    memset(&display_values, 0, sizeof(display_values));
    
    Serial.println("Setup completed successfully");
}

void loop() {
    static uint32_t last_ui_update = 0;
    const uint32_t UI_UPDATE_INTERVAL = 20;  // 50Hz refresh rate for UI
    lv_obj_t * active_screen = lv_scr_act();
    


    // Process CAN messages
    GetCanMessages();

    // Update UI at fixed interval
    uint32_t current_time = millis();
    if (current_time - last_ui_update >= UI_UPDATE_INTERVAL) {
        updateAllDisplayValues();
        lv_timer_handler();
        last_ui_update = current_time;
    }
}



