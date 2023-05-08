#include <class_pres3_copy.h>
#include <EEPROM.h>

//staticの実態を定義
float pres::ground_pres = 1013.13;
float pres::ground_alt = 0;
float pres::ground_T = 30;
bool pres::judge_updown = false;


pres::~pres()
{
  Serial.print("デストラクター");
}

pres::pres()
{

  Wire.begin();
    //BME280動作設定
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CONFIG);//動作設定
  Wire.write(0x00);//「単発測定」、「フィルタなし」、「SPI 4線式」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  //BME280測定条件設定
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CTRL_MEAS);//測定条件設定
  Wire.write(0x24);//「温度・気圧オーバーサンプリングx1」、「スリープモード」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了


  //BME280温度測定条件設定
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CTRL_HUM);//湿度測定条件設定
  Wire.write(0x01);//「湿度オーバーサンプリングx1」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  
  //BME280補正データ取得
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x88);//出力データバイトを「補正データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了


  Wire.requestFrom(BME280_ADDR, 26);//I2Cデバイス「BME280」に26Byteのデータ要求
  for (i = 0; i < 26; i++) {
    while (Wire.available() == 0 ) {}  //ここで止まっている
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  dig_T1 = ((uint16_t)((dac[1] << 8) | dac[0]));
  dig_T2 = ((int16_t)((dac[3] << 8) | dac[2]));
  dig_T3 = ((int16_t)((dac[5] << 8) | dac[4]));

  dig_P1 = ((uint16_t)((dac[7] << 8) | dac[6]));
  dig_P2 = ((int16_t)((dac[9] << 8) | dac[8]));
  dig_P3 = ((int16_t)((dac[11] << 8) | dac[10]));
  dig_P4 = ((int16_t)((dac[13] << 8) | dac[12]));
  dig_P5 = ((int16_t)((dac[15] << 8) | dac[14]));
  dig_P6 = ((int16_t)((dac[17] << 8) | dac[16]));
  dig_P7 = ((int16_t)((dac[19] << 8) | dac[18]));
  dig_P8 = ((int16_t)((dac[21] << 8) | dac[20]));
  dig_P9 = ((int16_t)((dac[23] << 8) | dac[22]));

  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0xE1);//出力データバイトを「補正データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  delay(1000);//1000msec待機(1秒待機)

  Serial.println("start pres");
}

void pres::set_value1()
{
/*
値取得関数、以下コピペ
*/
//  BME280測定条件設定(1回測定後、スリープモード)
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CTRL_MEAS);//測定条件設定
  Wire.write(0x25);//「温度・気圧オーバーサンプリングx1」、「1回測定後、スリープモード」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  delay(10);//10msec待機

  //測定データ取得
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0xF7);//出力データバイトを「気圧データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(BME280_ADDR, 8);//I2Cデバイス「BME280」に8Byteのデータ要求
  for (i=0; i<8; i++)
  {
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }

  adc_P = ((uint32_t)dac[0] << 12) | ((uint32_t)dac[1] << 4) | ((dac[2] >> 4) & 0x0F);
  adc_T = ((uint32_t)dac[3] << 12) | ((uint32_t)dac[4] << 4) | ((dac[5] >> 4) & 0x0F);

  pres_cal = BME280_compensate_P_int32(adc_P);//気圧データ補正計算
  temp_cal = BME280_compensate_T_int32(adc_T);//温度データ補正計算

  pressure = (float)pres_cal / 100.0;
  temp = (float)temp_cal / 100.0;


  if (last_alt == 0)
  //もしインスタンス作成直後だった場合は初期値を設定
  {
    last_alt =  ground_alt + R * (ground_T + 273.15) * log(ground_pres/pressure) / g;
  }

  //LPF（一時遅れ系にした）いつかはカルマンフィルター実装する！！
  float alt_temp =  ground_alt + R * (ground_T + 273.15) * log(ground_pres/pressure) / g; //高度を計算
  alt = alt_temp * (1.0 - filter) + last_alt * filter;
  last_alt = alt;
}

void pres::set_value()
/*
10回平均を取る
*/
{
  float sum = 0.0;
  for (int i = 0; i<10; i++)
  {
    set_value1();
    sum += alt;
  }
  alt = sum / 10.0;
}

void pres::dummy_set_ground_value(float p, float T, float h)
{

  /*
  地上での初期値を保存
  static変数を扱うために実装
  */
  ground_pres = p;
  ground_T = T;
  ground_alt = h;
}


void pres::set_ground_value()
{
  /*
  地上での初期値を保存
  static変数を扱うために実装
  最初の値はぶれやすいのでここで50回捨てる*/
  for (int i=0 ;i<50;i++)
  {
    set_value1();
    delay(100);
  }
  float pres_sum = 0.0;
  float temp_sum = 0.0;
  for (int i = 0; i < 50; i++)
  {
    pres_sum += pressure;
    temp_sum += temp;
  }
  float pres_ave = pres_sum / 50.0;
  float temp_ave = temp_sum / 50.0;
  alt = 0.0;
  last_alt = 0.0;
  Serial.print("ground value");
  Serial.println(pres_ave);
  Serial.print("temp");
  Serial.println(temp_ave);
  dummy_set_ground_value(pres_ave, temp_ave, 0);
}


void pres::set_judge_updown()
{
  /*
  judge_alt_to_groundで最低高度を上回ったらtrueにするために利用
  */
  judge_updown = true;
}

void pres::set_judge_updown2()
{
  /*
  judge_alt_to_groundで最低高度を上回ったらtrueにするために利用
  */
  judge_updown = false;
}


void pres::set_pre_alt()
{
  /*
  比較ようの値を保存
  */
  set_value();
  pre_alt = alt;
}


int32_t pres::BME280_compensate_T_int32(int32_t adc_T)
/*
温度補正関数、以下コピペ
*/
{
  int32_t var1, var2, T;
  var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T  = (t_fine * 5 + 128) >> 8;
  return T;
}

uint32_t pres::BME280_compensate_P_int32(int32_t adc_P)
{
/*
圧力補正関数、以下コピペ
*/
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
  var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}


float pres::get_ground_value()
{
  return ground_alt;
}


float pres::get_pres()
{
  return pressure;
}


float pres::get_alt()
{
  return alt;
}


bool pres::judge_alt_up()
{
  /*
  　地上からの上昇を検知
  */
  set_value();
  if (alt > pre_alt + marge1) 
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool pres::judge_alt_down()
{
  /*
  　上空からの落下を検知
  */
  set_value();
  if (alt < pre_alt - marge2) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool pres::judge_alt_to_ground()
{
  /*
  　最低高度を下回ったかどうかを判断
  */
  //2回目ground_alt + minnimam_heightに達するまでfalse
  set_value();

  if (alt > ground_alt + minnimam_height + marge) 
  {
    if (judge_updown)
    {
      Serial.print("rising2:");
      Serial.println(alt);
      return false;
      //三番目にここ
    }
    else
    {
      for (int i= 0; i < check_times; i++)
      {
        set_value();
        if (!(alt > ground_alt + minnimam_height + marge))
        {
          return false;
        }
      }
      Serial.print("first rise:");
      Serial.println(alt);
      set_judge_updown();
      delay(riseing_time_ms); //ちゃんと昇るまで待機
      return false;
      //2番目にここ
    }
  }

  else if (alt < ground_alt + minnimam_height - marge)
  {
    if (judge_updown)
    {
      for (int i =0 ; i < check_times; i++)
      {
        set_value();
        if (!(alt < ground_alt + minnimam_height - marge))
        {
          return false;
        }
      }
      Serial.println("reach target height");
      Serial.print("target alt:");
      Serial.println(minnimam_height);
      Serial.print("current alt:");
      Serial.println(alt);
      set_judge_updown2(); //次使う時用
      return true;
      //最後にここ
    }
    else
    {
      Serial.print("staying:");
      Serial.println(alt);
      return false;
      //最初はここ
    }
  }
  else 
  {
      Serial.print("rising1:");
      Serial.println(alt);
     return false;
  }
}
