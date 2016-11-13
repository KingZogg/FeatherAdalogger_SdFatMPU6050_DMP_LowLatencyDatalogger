#ifndef UserDataType_h
#define UserDataType_h
struct data_t {
  unsigned long time;
  int16_t aaWorldx;
  int16_t aaWorldy;
  int16_t aaWorldz;

 

  float temperature;
  float altitude;
  float humidity;

  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t mx;
  int16_t my;
  int16_t mz;
  float measuredvbat;
  int16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  int16_t logFifo;
};
#endif  // UserDataType_h