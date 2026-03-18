#include <Arduino.h>
#include <CANCREATE.h>
#include <SPICREATE.h>
#include <LPS25HB.h>
#include <ICM42688.h>
#include <SPIflash.h>

#define MIN1 9
#define MIN2 10
#define LED 11
#define CAN_RX 47
#define CAN_TX 48
#define SPI_SCK 17
#define SPI_MISO 16
#define SPI_MOSI 18
#define LPS_CS 7
#define ICM_CS 8
#define Flash_CS 15

#define SPI_FREQ 12000000
#define LPS_WAI 0xBD
#define ICM_WAI 0x47

bool Standby = false;
bool Standby_Prev = false;
bool Liftoff = false;
bool Liftoff_Prev = false;
bool Top = false;
bool Top_Prev = false;
bool Motor = false;
bool Motor_Prev = false;
bool Motor_Rev = false;
bool Motor_Rev_Prev = false;
bool Log = false;
bool Log_Prev = false;

uint16_t count_LPS = 1;
uint16_t count_ICM = 1;
uint16_t count_Motor = 0;
uint16_t count_Liftoff_Press = 0;
uint16_t count_Liftoff_Acc = 0;
uint16_t count_Top_Press = 0;
uint16_t count_Top_Time = 0;
uint16_t count_CAN = 0;
uint16_t count_Log = 0;
uint16_t count_LED = 0;
uint8_t LPS_data[3];
uint32_t Press_Avr = 0;
uint32_t Press_Avr_Prev = 0;
uint32_t Press_Avr_Temp = 0;
int32_t Press_Avr_Delta = 0;
int16_t ICM_data[6];
int16_t Acc_x_Avr = 0;
int32_t Acc_x_Avr_Temp = 0;
int16_t Acc_y_Avr = 0;
int32_t Acc_y_Avr_Temp = 0;
int16_t Acc_z_Avr = 0;
int32_t Acc_z_Avr_Temp = 0;
uint16_t Acc_Avr_Abs = 0;
uint8_t Flash_Write[256];
uint8_t Flash_Read[256];
uint32_t Logtime_Read = 0;
uint32_t Press_Read = 0;
int16_t Acc_x_Read = 0;
int16_t Acc_y_Read = 0;
int16_t Acc_z_Read = 0;
uint8_t Send_Liftoff[1] = {1};
uint8_t Send_Top[1] = {1};
uint32_t Logtime = 0;
uint8_t Log_Point = 0;
uint32_t Flash_Address = 0x100;

hw_timer_t *timer = NULL;

CAN_CREATE CAN(true);

can_setting_t can_setting = {
  .baudRate = (long)100E3,
  .multiData_send = true,
  .filter_config = CAN_FILTER_DEFAULT,
};

SPICREATE::SPICreate spi;
LPS lps;
ICM icm;
Flash flash;

IRAM_ATTR void counter() {
  if (lps.WhoAmI() == LPS_WAI) {
    if ((count_LPS % 40) == 0) {
      lps.Get(LPS_data);
      Press_Avr_Temp += LPS_data[0] + LPS_data[1] * 256 + LPS_data[2] * 65536;
    }
    if (count_LPS == 200) {
      Press_Avr = Press_Avr_Temp / 5;
      if (Standby) {
        Press_Avr_Delta = Press_Avr - Press_Avr_Prev;
        if (Press_Avr_Delta * 10 <= -4096) {
          count_Liftoff_Press++;
        }
        else {
          count_Liftoff_Press = 0;
        }
        if (Liftoff) {
          if (Press_Avr >= Press_Avr_Prev) {
            count_Top_Press++;
          }
          else{
            count_Top_Press = 0;
          }
        }
      }
      Press_Avr_Prev = Press_Avr;
      Press_Avr_Temp = 0;
      count_LPS = 0;
    }
  }
  if (icm.WhoAmI() == ICM_WAI) {
    icm.Get(ICM_data);
    Acc_x_Avr_Temp += ICM_data[1];
    Acc_y_Avr_Temp += ICM_data[2];
    Acc_z_Avr_Temp += ICM_data[3];
    if (count_ICM == 20) {
      Acc_x_Avr = Acc_x_Avr_Temp / 20;
      Acc_y_Avr = Acc_y_Avr_Temp / 20;
      Acc_z_Avr = Acc_z_Avr_Temp / 20;
      if (Standby) {
        Acc_Avr_Abs = (Acc_x_Avr / 2048) ^ 2 + (Acc_y_Avr / 2048) ^ 2 + (Acc_z_Avr / 2048) ^ 2;
        if (Acc_Avr_Abs >= 4) {
          count_Liftoff_Acc++;
        }
        else{
          count_Liftoff_Acc = 0;
        }
      }
      Acc_x_Avr_Temp = 0;
      Acc_y_Avr_Temp = 0;
      Acc_z_Avr_Temp = 0;
      count_ICM = 0;
    }
  }
  if (Standby) {
    if (Liftoff) {
      count_Top_Time++;
    }
    count_CAN++;
    count_LED++;
  }
  if (Motor or Motor_Rev) {
    count_Motor++;
  }
  if (Log) {
    count_Log++;
    Logtime++;
  }
  count_LPS++;
  count_ICM++;
}

void setup() {
  pinMode(MIN1, OUTPUT);
  pinMode(MIN2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LPS_CS, OUTPUT);
  pinMode(ICM_CS, OUTPUT);
  pinMode(Flash_CS, OUTPUT);
  digitalWrite(MIN1, LOW);
  digitalWrite(MIN2, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(LPS_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);
  digitalWrite(Flash_CS, HIGH);
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);
  Serial.println("CAN Sender");
  if (CAN.begin(can_setting, CAN_RX, CAN_TX)) {
    Serial.println("Starting CAN failed!");
  }
  delay(100);
  switch (CAN.test())
  {
  case CAN_SUCCESS:
    Serial.println("Success!!!");
    break;
  case CAN_UNKNOWN_ERROR:
    Serial.println("Unknown error occurred");
    break;
  case CAN_NO_RESPONSE_ERROR:
    Serial.println("No response error");
    break;
  case CAN_CONTROLLER_ERROR:
    Serial.println("CAN CONTROLLER ERROR");
    break;
  default:
    break;
  }
  spi.begin(SPI2_HOST, SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.println("SPI Start");
  lps.begin(&spi, LPS_CS, SPI_FREQ);
  Serial.println("LPS Start");
  icm.begin(&spi, ICM_CS, SPI_FREQ);
  Serial.println("ICM Start");
  flash.begin(&spi, Flash_CS, SPI_FREQ);
  Serial.println("Flash Start");
  timer = timerBegin(0, getApbFrequency() / 1000000, true);
  timerAttachInterrupt(timer, &counter, false);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

void loop() {
  uint8_t Rot_Data[6] = {(uint8_t)(ICM_data[4] >> 8), (uint8_t)ICM_data[4], (uint8_t)(ICM_data[5] >> 8), (uint8_t)ICM_data[5], (uint8_t)(ICM_data[6] >> 8), (uint8_t)ICM_data[6]};
  uint8_t Acc_Data[6] = {(uint8_t)(ICM_data[1] >> 8), (uint8_t)ICM_data[1], (uint8_t)(ICM_data[2] >> 8), (uint8_t)ICM_data[2], (uint8_t)(ICM_data[3] >> 8), (uint8_t)ICM_data[3]};
  uint8_t Press_Data[3] = {LPS_data[0], LPS_data[1], LPS_data[2]};
  if (CAN.available()) {
    can_return_t Data;
    if (CAN.readWithDetail(&Data)){
      Serial.println("failed to get CAN data");
    }
    if (Data.id == 0x00a) {
      Standby = false;
    }
    if (Data.id == 0x005) {
      Standby = true;
    }
    if (Data.id == 0x00d) {
      Motor = true;
      Motor_Rev = false;
    }
    if (Data.id == 0x00f) {
      Motor_Rev = true;
      Motor = false;
    }
    if (Data.id == 0x003) {
      Motor = false;
      Motor_Rev = false;
    }
    if (Data.id == 0x011) {
      Log = true;
    }
    if (Data.id == 0x01e) {
      Log = false;
    }
  }
  if (count_CAN >= 200) {
    CAN.sendData(0x120, Rot_Data, 6);
    CAN.sendData(0x11a, Acc_Data, 6);
    CAN.sendData(0x10a, Press_Data, 3);
    count_CAN = 0;
  }
  if (Liftoff and !Liftoff_Prev) {
    CAN.sendData(0x110, Send_Liftoff, 1);
  }
  if (Top and !Top_Prev) {
    CAN.sendData(0x12a, Send_Top, 1);
  }
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.println(cmd);
    if (cmd == 'e') {
      Standby = false;
    }
    if (cmd == 's') {
      Standby = true;
    }
    if (cmd == 'p') {
      Motor = true;
      Motor_Rev = false;
    }
    if (cmd == 'r') {
      Motor_Rev = true;
      Motor = false;
    }
    if (cmd == 'a') {
      Motor = false;
      Motor_Rev = false;
    }
    if (cmd == 'l') {
      Log = true;
    }
    if (cmd == 'm') {
      Log = false;
    }
    if (cmd == 'o') {
      Liftoff = true;
    }
    if (cmd == 't') {
      Top = true;
    }
    if (cmd == 'w') {
      timerAlarmDisable(timer);
      for (uint32_t i = 0x100; i < Flash_Address; i += 0x100) {
        flash.read(i, Flash_Read);
        for (uint8_t j = 0; j <= 15; j++) {
          Logtime_Read = (uint32_t)(Flash_Read[j * 16] * 16777216 + Flash_Read[j * 16 + 1] * 65536 + Flash_Read[j * 16 + 2] * 256 + Flash_Read[j * 16 + 3]);
          Press_Read = (uint32_t)(Flash_Read[j * 16 + 4] + Flash_Read[j * 16 + 5] * 256 + Flash_Read[j * 16 + 6] * 65536);
          Acc_x_Read = (int16_t)(Flash_Read[j * 16 + 7] << 8 + Flash_Read[j * 16 + 8]);
          Acc_y_Read = (int16_t)(Flash_Read[j * 16 + 9] << 8 + Flash_Read[j * 16 + 10]);
          Acc_z_Read = (int16_t)(Flash_Read[j * 16 + 11] << 8 + Flash_Read[j * 16 + 12]);
          Serial.print((uint32_t)Logtime_Read);
          Serial.print(",");
          Serial.print((uint32_t)Press_Read);
          Serial.print(",");
          Serial.print((int16_t)Acc_x_Read);
          Serial.print(",");
          Serial.print((int16_t)Acc_y_Read);
          Serial.print(",");
          Serial.println((int16_t)Acc_z_Read);
        }
      }
      timerAlarmEnable(timer);
    }
    if (cmd == 'x') {
      Serial.println("Start Erase");
      flash.erase();
      Serial.println("Erased");
    }
  }
  if (Standby) {
    if (!Standby_Prev) {
      digitalWrite(LED, HIGH);
      Serial.println("Standby On");
    }
    if ((count_Liftoff_Press >= 5) or (count_Liftoff_Acc >= 50)) {
      Liftoff = true;
    }
    if (Liftoff) {
      if (!Liftoff_Prev) {
        Serial.println("Liftoff");
      }
      if (count_LED >= 1000) {
        digitalWrite(LED, HIGH);
      }
      if (count_LED >= 2000) {
        digitalWrite(LED, LOW);
        count_LED = 0;
      }
    }
    if ((count_Top_Press >= 5) or (count_Top_Time >= 14000)) {
      Top = true;
    }
    if (Top) {
      if (!Top_Prev) {
        Serial.println("Top");
        Motor = true;
      }
      if (count_LED >= 500) {
        digitalWrite(LED, HIGH);
      }
      if (count_LED >= 1000) {
        digitalWrite(LED, LOW);
        count_LED = 0;
      }
    }
    if (count_Motor >= 2500) {
      Motor = false;
      Motor_Rev = false;
      count_Motor = 0;
    }
    if (Motor) {
      if (!Motor_Prev) {
        Serial.println("Motor On");
      }
      digitalWrite(MIN1, LOW);
      digitalWrite(MIN2, HIGH);
    }
    if (Motor_Rev) {
      if (!Motor_Rev_Prev) {
        Serial.println("Motor Reverse On");
      }
      digitalWrite(MIN1, HIGH);
      digitalWrite(MIN2, LOW);
    }
    if (!(Motor or Motor_Rev)) {
      if (Motor_Prev or Motor_Rev_Prev) {
        Serial.println("Motor Off");
      }
      digitalWrite(MIN1, LOW);
      digitalWrite(MIN2, LOW);
    }
    if (Log) {
      if (!Log_Prev) {
        Serial.println("Loging Start");
        Logtime = 0;
      }
      if (count_Log >= 500) {
        for (uint8_t i = 0; i <= 3; i++) {
          Flash_Write[Log_Point * 16 + i] = Logtime >> ((3 - i) * 8);
        }
        for (uint8_t i = 4; i <= 6; i++) {
          Flash_Write[Log_Point * 16 + i] = Press_Data[i - 4];
        }
        for (uint8_t i = 7; i <= 12; i++) {
          Flash_Write[Log_Point * 16 + i] = Acc_Data[i - 7];
        }
        for (uint8_t i = 13; i <= 15; i++) {
          Flash_Write[Log_Point * 16 + i] = 0;
        }
        Log_Point++;
        if (Log_Point == 16) {
          flash.write(Flash_Address, Flash_Write);
          Flash_Address += 0x100;
          Log_Point = 0;
        }
        count_Log = 0;
      }
    }
    if (!Log and Log_Prev) {
      Serial.println("Loging Stop");
      Logtime = 0;
    }
  }
  else {
    if (Standby_Prev) {
      Serial.println("Standby Off");
    }
    Liftoff = false;
    Top = false;
    Motor = false;
    Motor_Rev = false;
    Log = false;
    count_Liftoff_Press = 0;
    count_Liftoff_Acc = 0;
    count_Top_Press = 0;
    count_Top_Time = 0;
    Logtime = 0;
    digitalWrite(MIN1, LOW);
    digitalWrite(MIN2, LOW);
    digitalWrite(LED, LOW);
  }
  Standby_Prev = Standby;
  Liftoff_Prev = Liftoff;
  Top_Prev = Top;
  Motor_Prev = Motor;
  Motor_Rev_Prev = Motor_Rev;
  Log_Prev = Log;
}