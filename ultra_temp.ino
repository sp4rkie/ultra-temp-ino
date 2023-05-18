/*
 * battery life estimation:
 *  - 38mA every 10min for 3.8s
 *  - background current: 6uA
 *  - batt cap: 1400mAh
 * 
 *   3.8s @ 38mA:
 *   1400mAh / (38mA * 3.8s / 3600) = 34903.047 (updates possible with cap)
 *      => 
 *   34903 * 10min = 349030min = 5817h = 242days 
 *
 *   background current consideration:
 *   1400mAh / 0.006mA = 233333h = 9722days = 26years
 *
 *   combined life time (active + background):
 *   242 * 9722 / (242 + 9722) = 236 days
 *
 * variations:
 *
 *   3.8s @ 45mA:                    
 *   1400mAh / (45mA * 3.8s / 3600) = 29473.6842 (updates possible with cap)
 *      => 
 *   29473 * 10min = 294730min = 4912h = 204days
 *
 *   1.8s @ 38mA:                    
 *   1400mAh / (38mA * 1.8s / 3600) = 73684.210 (updates possible with cap)
 *      => 
 *   73684 * 10min = 736840min = 12280h = 511days = 1.4years
 *
 *   1.6s @ 38mA:                    
 *   1400mAh / (38mA * 1.6s / 3600) = 82894.737 (updates possible with cap)
 *      => 
 *   82894 * 10min = 828947min = 13815h = 575days = 1.6years
 *
 */

#ifdef ESP32_4                          // -> new test dev
#define DEBUG   1
#define VBAT_ADC1_GND_PIN    22              // use a dynamic ground to effectively void its deep sleep current
#define VBAT_ADC1_SENSE_PIN  36
#else
#define DEBUG   1
#define VBAT_ADC1_GND_PIN    22              // use a dynamic ground to effectively void its deep sleep current
#define VBAT_ADC1_SENSE_PIN  36
#endif

#define TEMPRA 32                       // one wire pin
#define TIME_TO_SLEEP  600000000        // 600s == 10min
// max:               4294967295        // 0xFFFFFFFF

#include "mlcf.h"
#ifdef MCFG_LOCAL
#include "mcfg_local.h"
#include "mcfg_locals.h"
#else
#include "mcfg.h"
#endif
#include "mota.h"
#include <esp_wifi.h>                   // required by esp_wifi_stop()

#ifdef TEMPRA
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(TEMPRA);
DallasTemperature sensors(&oneWire);
#endif

#if defined(ESP32_1)    // -> new USY
#define BIG_DISPLAY
#elif defined(ESP32_3)
#elif defined(ESP32_4)  // -> new test dev
#define BIG_DISPLAY  
//#define TEST_BBOX
#elif defined(ESP32_5)
#elif defined(ESP32_6)
#elif defined(ESP32_7)
#elif defined(ESP32_8)
#else 
this may not happen
#endif

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMono9pt7b.h>

#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#ifdef BIG_DISPLAY
// 2.9" 296x128 
#define GxEPD2_DRIVER_CLASS GxEPD2_290_T94_V2 // GDEM029T94  128x296, SSD1680, Waveshare 2.9" V2 variant
#else
// 2.13" 250×122   
#define GxEPD2_DRIVER_CLASS GxEPD2_213_BN // DEPG0213BN  128x250, SSD1680, TTGO T5 V2.4.1, V2.3.1
#endif

#define MAX_DISPLAY_BUFFER_SIZE 65536ul 
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? \
                         EPD::HEIGHT : \
                         MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
#define FULL_REFRESH 12  // every other 12 x 10min = 2h

// our cfg:
// BUSY     27      #                                           (free selectable)
// RST      33      #                                           (free selectable)
// DC       25      # = data/command                            (free selectable)
// CS       26      # = SS (SLAVE SELECT) == CS (CHIP SELECT)   (free selectable)
// CLK      18      # = SCK                                     (hardwired)
// DIN      23      # = MOSI                                    (hardwired)
// GND      GND     # 
// VCC      3V3     # 
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(26, 25, 33, 27));

/*
 * FreeMonoBold24pt7b samples:
 *
 * how to calculate fillRect() and setCursor() from posx, posy
 *
 * formula:
 * display.getTextBounds("text", posx, posy, &tbx, &tby, &tbw, &tbh)
 * display.fillRect(tbx, tby + tbh, tbw, tbh, ffff)
 * display.setCursor(posx, posy + tbh)
 *
 *  ==>
 *      tbx = posx +  2
 *      tby = posy - 30
 *  ==>
 *      display.fillRect(posx +  2, posy - 30 + 34, 135, 34, ffff)
 *      display.fillRect(posx +  2, posy +       4, 135, 34, ffff)
 *      display.setCursor(posx,     posy + 34)
 *  ==>
 *      display.fillRect(posx + 2,  posy +  4,      135, 34, ffff)
 *      display.setCursor(posx,     posy + 34)
 *  ==>
 *      display.fillRect(posx + 2,  posy - 30,      135, 34, ffff)
 *      display.setCursor(posx,     posy)
 *  ==>
 *      display.fillRect(posx,      posy,           135, 34, ffff)
 *      display.setCursor(posx - 2, posy + 30)
 *
 * sample (leftmost upper edge possible (for FreeMonoBold24pt7b)):
 * posx == -2
 * posy == -4
 * display.getTextBounds(err0, -2, -4, &tbx, &tby, &tbw, &tbh); // ==> 0 -34 135 34
 * display.fillRect(0, 0, 135, 34, ffff)          
 * display.setCursor(-2, 30)
 *
 * sample:
 * posx == 0
 * posy == 0
 * display.getTextBounds(err0, 0, 0, &tbx, &tby, &tbw, &tbh);   // ==> 2 -30 135 34
 * display.fillRect(2, 4, 135, 34, ffff)
 * display.setCursor(0, 34)
 *
 * sample:
 * posx == 10
 * posy == 20
 * display.getTextBounds(err0, 10, 20, &tbx, &tby, &tbw, &tbh); // ==> 12 -10 135 34
 * display.fillRect(12, 24, 135, 34, ffff)
 * display.setCursor(10, 54)
 */
#define D_WIDTH  display.width()
#define D_HEIGHT display.height()

_u16 ep_x, ep_y0, ep_y1;
_u16 tbw, tbh;
_i16 tbx, tby; 

void
deep_sleep(_u32 time_to_sleep)
{
    display.hibernate(); // avoid writing crap to display
if (DEBUG) Serial.printf("sleeping for %us\n", time_to_sleep / 1000000);
    esp_sleep_enable_timer_wakeup(time_to_sleep);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
}

void
setup()
{
    Serial.begin(115200);
#ifdef TEMPRA
    sensors.begin();                                        // async start, don't move
    sensors.setWaitForConversion(0);
    sensors.requestTemperatures();
#endif
#if defined(ESP32_1)    // -> new USY
    mysetup_intro(ROTA5G_SSID, __FILE__, 0);
#else
    mysetup_intro(ROTA2G_SSID, __FILE__, 0);
#endif
    display.init(DEBUG > 5 ? 115200 : 0, 0);                // false == partial 
    display.setRotation(3);                                 // rotation 1, 3 is landscape both
    display.setFont(&FreeMonoBold24pt7b);
    display.setTextColor(GxEPD_WHITE);

    // FreeMonoBold24pt7b.h
    display.getTextBounds("00000", 0, 0, &tbx, &tby, &tbw, &tbh);   // choose some text suitable to calc the max bounding box
    ep_x = (D_WIDTH - tbw) / 2 - tbx;              
    ep_y0 = (D_HEIGHT - tbh * 2) / 3     - tby;
    ep_y1 = (D_HEIGHT - tbh * 2) / 3 * 2 - tby + tbh;

    pinMode(VBAT_ADC1_GND_PIN, OUTPUT);
    digitalWrite(VBAT_ADC1_GND_PIN, 0);
}

RTC_DATA_ATTR _i32 _RSSI;
RTC_DATA_ATTR _u32 _TP04;
RTC_DATA_ATTR _u32 _TP05;
RTC_DATA_ATTR _u32 _UBAT;

void 
loop() 
{
    _i8p temp_ext;
    _i8 temp_int[10];
    _u8 cnt;
    _i8 cmd[64];

#ifdef TEMPRA
    /*
     * new DS18B20 sporadically spits like this?!
     * 
     * +#.WAIT -127.00
     * WAIT 85.00
     * WAIT 85.00
     * WAIT 23.19
     * temp_int: 23.19
     * 
     * quick and dirty hack
     */
    cnt = bootCount == 1 ? 30 : 10; // takes often more than 16 retries after first power on
    do {
        sprintf(temp_int, "%.2f", sensors.getTempCByIndex(0));
        Serial.printf("GOT: %s\n", temp_int);   // don't remove to provide some intentional extra delay (no delay() here)
    } while ((!strcmp("85.00", temp_int) || !strcmp("-127.00", temp_int)) && --cnt);
    if (!cnt) {
        Serial.printf("temp not plausible\n");
        sprintf(temp_int, "%s", _err[7]);
    } else if (sensors.isConversionComplete()) {
//        sprintf(temp_int, "%.2f", sensors.getTempCByIndex(0));
    } else {
        Serial.printf("temp conversion not complete\n");
        sprintf(temp_int, "%s", _err[6]);
    }
#endif

#ifdef TEMPRA
//  snprintf(cmd, _SZ(cmd), "@temp" "@temp_" HOST "=" "%s", temp_int);
    snprintf(cmd, _SZ(cmd), "@temp" "@temp_" HOST "=" "%s/%d/%u/%u/%u", 
                                                      temp_int, 
                                                      _RSSI, 
                                                      _TP04, 
                                                      _TP05, 
                                                      _UBAT);
#else
    snprintf(cmd, _SZ(cmd), "@temp");
#endif
    if (mysend(cmd, STD_TARGET_HOST, STD_TARGET_PORT, &temp_ext)) {
if (DEBUG) Serial.printf("mysend failed\n");
    }
    /*
     * 4069 AT 2.90V VCC
     * 3984 AT 2.80V VCC
     * 3839 AT 2.70V VCC
     */
if (DEBUG) Serial.printf("<RSSI: %d>\n", _RSSI = WiFi.RSSI());
if (DEBUG) Serial.printf("<UBAT: %umV>\n", _UBAT = analogReadMilliVolts(VBAT_ADC1_SENSE_PIN));
if (DEBUG) Serial.printf("<TP04: %u>\n", _TP04 = millis());   // typical TP04: 253 on esp32_1
    if (strcmp(temp_ext, "OTA")) {
        esp_wifi_stop();
    }
#ifdef TEST_BBOX
    // print BBOX for a specific string calculated in getTextBounds()
    display.setTextColor(GxEPD_BLACK);
    display.fillRect(ep_x + tbx, ep_y0 + tby, tbw, tbh, GxEPD_WHITE);
    display.fillRect(ep_x + tbx, ep_y1 + tby, tbw, tbh, GxEPD_WHITE);
#endif
    display.setCursor(ep_x, ep_y0);
    display.print(temp_ext);
#ifdef TEMPRA
    display.setCursor(ep_x, ep_y1);
    display.print(temp_int);
#endif

#define INTRA_STR_OFFS 17
#define DISP_HEIGHT_OFFS 5

    display.setFont(&FreeMono9pt7b);
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(2, D_HEIGHT - DISP_HEIGHT_OFFS);
    display.print(cmd + INTRA_STR_OFFS);

    display.display(bootCount != 1 && bootCount % FULL_REFRESH);    // false == full
    if (!strcmp(temp_ext, "OTA")) {
        myota(OTA_PERIOD);
    }
if (DEBUG) Serial.printf("<TP05: %u>\n", _TP05 = millis());   // typical TP05: 1390 on esp32_1
#if defined(ESP32_4) && DEBUG > 1       // -> new test dev
    deep_sleep(10000000);
#else
    deep_sleep(TIME_TO_SLEEP);
#endif
}

