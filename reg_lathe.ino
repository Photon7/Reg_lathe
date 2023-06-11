#define Flash_prog 0  //  Программируем память или регулятор
#define menu_debug 0  //  Проверяем меню

#if (Flash_prog == 0 && menu_debug == 0)
#include <SPIMemory.h>
#include <EncButton2.h>
#include <GyverPID.h>
#include <PIDtuner.h>
#include <PIDtuner2.h>
#include <GyverTimers.h>
#include <GyverWDT.h>
#include <GyverFilters.h>
#include <GyverDimmer.h>
#include <GyverNTC.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <mString.h>
#endif

#if (Flash_prog == 0 && menu_debug == 0 || menu_debug == 1)
#include <LiquidMenu.h>
#endif

#if (Flash_prog == 0 && menu_debug == 0)
#define CLK 3
#define DT 4
#define SW 5
#define D_PIN 17  // пин управления симистором
#define ITEMS 12  // Общее кол во пунктов
#define _LCD_TYPE 1

// #include <LCD_1602_RUS_ALL.h>
//#include <font_LCD_1602_RUS.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);  // пины экрана

GyverPID regulator(0.0, 0.00, 0.00, 10);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
Dimmer<D_PIN> dim;                        // указать пин диммера

PIDtuner tuner;
GMedian3<uint32_t> median_taho;

EncButton2<EB_ENCBTN> enc(INPUT, CLK, DT, SW);  // энкодер с кнопкой

GyverNTC therm_engine(0, 10000, 3950);    // пин, R термистора при 25 градусах, B термистора. (R резистора по умолч. 10000)
GyverNTC therm_heatsink(0, 10000, 3950);  // пин, R термистора при 25 градусах, B термистора. (R резистора по умолч. 10000)

mString<150> str_flash;
mString<80> str_display;

char printBuffer[128];
#endif

uint32_t spi_addr;
String outputString;

SPIFlash flash;

#if (Flash_prog == 0 && menu_debug == 0)
uint32_t engine_rpm;
uint32_t last_oboroti_po_regulytoru_dvigately;
uint32_t Time_LCD;                         //    временные переменные для таймера экрана
uint32_t oboroti_po_regulytoru_dvigately;  //    показания регулятора для расчёта
uint32_t oboroti_po_pokazaniym_taho;       //    обороты по датчику двигателя
uint32_t Time_WDT = 0;
uint32_t triac_breakdown_timer;  //   таймер по отключению реле после входа в аварийный режим
uint32_t capacitor_timer;        //   таймер зедержки для разряда конденсатора
uint32_t relay_timer;
uint32_t prsTimer;                                //    таймер парсинга
uint32_t oboroti_po_pokazaniym_taho_temp;         //    переменная для проверки скрорости на пробой симистора
uint32_t oboroti_po_pokazaniym_taho_obstruction;  //    переменная для проверки скрорости на препятствие
uint32_t obstruction_check_timer;                 //    таймер для проверки падения скрости при препятствии
uint32_t obstruction_timer;                       //    таймер для задержки запоминания скрости для проверки препятствия

int obmstep = 5;
float step_gear = 0.01;
int step_pulley = 1;
int step_tuning_signal_value = 5;
int step_tuning_value = 5;
int step_tuning_period_value = 5;
float coeff_step = 0.01;

float coeff_Kp;
float coeff_Ki;
float coeff_Kd;

volatile long engpower_tmp = 150;        //    временная переменная для расчёта задержки.
volatile int holl = 0;                   //    переменная  срабатываня датчика
volatile int zerocross_isr;              //    переменная кратности прерывания по 0
volatile uint32_t interrupt_taho_timer;  //    таймер ограничения частоты расчета оборотов
volatile uint16_t int_tic;               //   переменные для подсчёта времени между импульсами переменная подсчета переполнений таймера
volatile uint32_t tic;                   //   переменные для подсчёта времени между импульсами
volatile uint16_t t = 100;               //   минимальное время импульсов 0,4906с. 100 переполнений таймера.
volatile uint32_t kol_Imp_tahoA;         //   количество импульсов на оборот двигателя с учетом передачи
volatile uint32_t holl_count;            //    переменная  срабатываня датчика для отображения в меню

uint32_t protection_timer;    //    таймер защите от превышения оборотов по двигателю
uint32_t speed_change_timer;  //    таймер ограничения переключения скорости

int tormoz = 30;                     //   переменная торможения 30(минимум)  255(максимум)  рекомендуется до 150
int zaschita;                        //   количество оборотов на которое можно превысить. затем срабатывает защита.        //  eeprom
int engpower;                        //   мощность двигателя
int accuracy;                        //   точность настройки пид регулятора
int eng_torm_start_power = 20;       //   стартовая мощность торможения двигателя
int eng_torm_full_power = 50;        //   максимальная мощность торможения двигателя
int enc_speed_step = 20;             //   шаг изменения скорости энкодером по умолчанию 20
int eng_speed_rate;                  //   Скорость двигателя     //      дописать в eeprom
int eng_pulse_quantity;              //   количество импульсов на оборот двигателя, измеряется по двигателю          // дописать в eeprom
int rele_1 = 20;                     //   Реле  на А6
int rele_2 = 21;                     //   Реле  на А7
int last_revers_state = 1;           //   предидущее положение тумблера реверс
int i = 0;                           //   переменная первой строки текстового массива
int b = 0;                           //   переменная второй строки текстового массива
int polozenie_tumblera_revers = 1;   //   положение тумблера реверс
int last_polozenie_tumblera_revers;  //   переменная для запоминания состояния режима двигателя
int last_text;                       //   переменная для текста при запоминании состояния режима двигателя
int obstruction_multiplicity;        //   переменная кратности для сброса флага препятствие
int eng_gear_ratioI[2];              //   массив количества импульсов на 10 оборотов двигателя скоростей 2, 3

uint32_t obMin[5];         //     массив минимальных скоростей 1-4
uint32_t obMax[5];         //     массив максимальных скоростей 1-4
uint32_t kol_Imp_taho[5];  //     массив количества импульсов на 10 оборотов двигателя скоростей 1-5

float eng_gear_ratioF[2];  //     массив количества импульсов на 10 оборотов двигателя скоростей 4, 5
float Qtr;                 //     коэффициент

uint32_t engminA = 1000;       //     минимальная скорость 1000 об/мин по умолчанию
uint32_t engmaxA = 10000;      //     максимальная скорость 10000 об/мин по умолчанию
uint32_t obMinA;               //     минимальная скорость по умолчанию об/мин
uint32_t obMaxA;               //     пределы скорости по умолчанию об/мин
uint32_t diameter_q4_pulley1;  //    Диаметр шкива ведущего 4 сокрости мм
uint32_t diameter_q4_pulley2;  //    Диаметр шкива ведомого 4 сокрости мм
uint32_t diameter_q5_pulley1;  //    Диаметр шкива ведущего 5 сокрости мм
uint32_t diameter_q5_pulley2;  //    Диаметр шкива ведомого 5 сокрости мм

uint32_t tuning_signal_value;
uint32_t tuning_step_value;
uint32_t tuning_period_value;
uint32_t tuning_accuracy_value;
uint32_t tuning_pulse_length;
uint32_t tuning_iteration_period;
int tuning_result_accuracy;

long quantity_value;


// флажки торможения
boolean tormozenie_flag;      //    Флаг торможения разрешено
boolean switched_flag;        //    Флаг переключения реле
boolean eng_to_stop_flag;     //    Флаг торможение включено
boolean eng_off_flag;         //    Флаг двигатель выключен
boolean eng_speed_down_flag;  //    Флаг сброса сокрости двигателя для торможения

boolean relay_flag;           //   Флаг разрещения переключение реле
boolean zerocross_flag;       //   Флаг перехода через ноль
boolean rew = false;          //   Флаг положения переключения реле
boolean multiplicity5_flag;   //   Флаг кратности 5 для торможения
boolean manual_control_flag;  //   Флаг ручного управления через порт

boolean tuning_flag;                     //   Флаг разрешения настройки двигателя
boolean in_tuning_flag;                  //   Флаг в процессе настройки двигателя
boolean tuning_setting_incomplete_flag;  //   Флаг установки для настройки двигателя начальные не выполнены
boolean tuner_print_flag = true;         //   Флаг настройки выывода данных при настройке двигателя  включен плоттер / выключен порт

// флажки двигателя
boolean eng_setting_incomplete_flag;        //   Флаг установки настроек двигателя
boolean eng_speed_setting_incomplete_flag;  //   Флаг установки настроек при запуске регулятора двигателя
boolean eng_rate_down_flag;                 //   Флаг изменения скорости вниз
boolean eng_rate_up_flag;                   //   Флаг изменения скорости вверх
boolean eng_state_flag;                     //   Флаг пуск / остановка
boolean was_clicked_flag;                   //   Флаг была нажата кнопка кнопка
boolean enc_speed_step_flag;                //   Флаг была изменен шаг энкодера
boolean enc_revers_left_flag;               //   Флаг режим двигателя изменен на влево
boolean enc_revers_right_flag;              //   Флаг режим двигателя изменен на вправо
boolean enc_speed_mode_flag;                //   Флаг режим скорости изменен на следующую
boolean tuning_dir_flag;                    //   Флаг направления настройки тюнинга
boolean menu_flag;                          //   Флаг вход в меню
boolean triac_breakdown_flag;               //   Флаг пробой симистора аварийный
boolean triac_breakdown_timer_flag;         //   Флаг пробой симистора аварийный временной для отображения
boolean recievedFlag;                       //   Флаг получены данные из порта
boolean eng_connect_flag;                   //   Флаг подключения двигателя
boolean eng_obstruction_detected_flag;      //   Флаг обнаружено препятствие
boolean eng_obstruction_check_timer_flag;   //   Флаг задержки проверки падения скорости
boolean obstruction_timer_flag;             //   Флаг задержки запоминания скорости
boolean tuning_complete_flag;               //   Флаг завершения настройки двигателя
#endif

#if (Flash_prog == 1)
boolean flash_programed_flag;  //   Флаг память запрограммирована
#endif

#if (Flash_prog == 0 && menu_debug == 0)
const char divider = ' ';
const char ending = ';';

const char *headers[] = {
  "Omi1",  //   1
  "Oma1",  //   2
  "Omi2",  //   3
  "Oma2",  //   4
  "Omi3",  //   5
  "Oma3",  //   6
  "Omi4",  //   7
  "Oma4",  //   8
  "Omi5",  //   9
  "Oma5",  //   10
  "Epl",   //   11
  "Im2",   //   12
  "Im3",   //   13
  "Im4",   //   14
  "Im5",   //   15
  "Est",   //   16
  "Rte",   //   17
  "Lft",   //   18
  "Rht",   //   19
  "On",    //   20
  "Off",   //   21
  "Stt",   //   22
  "Up",    //   23
  "Dn",    //   24
  "Menu",  //   25
  "Mn",    //   26
  "Mf",    //   27
  "Tun",   //   28
  "Dir",   //   29
  "Sig",   //   30
  "Stp",   //   31
  "Prd",   //   32
  "Acc",   //   33
  "Pls",   //   34
  "Itn",   //   35
  "Racc",  //   36
  "Tst",   //   37
  "Kp",    //   38
  "Ki",    //   39
  "Kd",    //   40
  "Clr",   //   41
  "Res",   //   42
  "Tpf1",  //   43
  "Tpf2",  //   44
  "Lst",   //   45

};



enum names {
  Omi1,  //   1     //    обороты минимальные 1 скорости по двигателю
  Oma1,  //   2     //    обороты максимальные 1 скорости по двигателю
  Omi2,  //   3     //    обороты минимальные 2 скорости с переменым коэффициентом целым
  Oma2,  //   4     //    обороты максимальные 2 скорости с переменым коэффициентом целым
  Omi3,  //   5     //    ..  3
  Oma3,  //   6     //    ..
  Omi4,  //   7     //    обороты минимальные 4 скорости с переменым коэффициентом по шкиву
  Oma4,  //   8     //    обороты максимальные 4 скорости с переменым коэффициентом по шкиву
  Omi5,  //   9     //    обороты минимальные 5 скорости с переменым коэффициентом по шкиву
  Oma5,  //   10    //    обороты максимальные 5 скорости с переменым коэффициентом по шкиву
  Epl,   //   11    //    количестов импульсов двигателя на 1 оборот
  Im2,   //   12    //    количество импульсов 2 скорости на 10 оборотов двигателя с переменым коэффициентом целым
  Im3,   //   13    //    количество импульсов 3 скорости на 10 оборотов двигателя с переменым коэффициентом целым
  Im4,   //   14    //    количество импульсов 4 скорости на 10 оборотов двигателя с переменым коэффициентом по шкиву
  Im5,   //   15    //    количество импульсов 5 скорости на 10 оборотов двигателя с переменым коэффициентом по шкиву
  Est,   //   16    //    настройки двигателя
  Rte,   //   17    //    обороты двигателя
  Lft,   //   18    //    режим двигателя влево
  Rht,   //   19    //    .. вправо
  On,    //   20    //    пуск двигателя
  Off,   //   21    //    остановка двигателя
  Stt,   //   22    //    пуск / остановка двигателя
  Up,    //   23    //    увеличить скорость двигателя
  Dn,    //   24    //    уменьшить скорость двигателя
  Menu,  //   25    //    вход в меню
  Mn,    //   26    //    ручное управление через порт включение
  Mf,    //   27    //    ручное управление через порт выключение
  Tun,   //   28    //    настройка коэффициентов двигателя
  Dir,   //   29    //    направление настройки, NORMAL / REVERSE
  Sig,   //   30    //    сигнал: базовый сигнал на управляющее устройство
  Stp,   //   31    //    тупенька: величина, на которую будет изменяться сигнал в обе стороны от базового
  Prd,   //   32    //    период: период опроса в ожидании стабилизации
  Acc,   //   33    //    точность стабилизации
  Pls,   //   34    //    продолж.импульса
  Itn,   //   35    //    период итерации
  Racc,  //   36    //    стабильность системы искомая
  Tst,   //   37    //    Текущие настройки тюнинга двигателя
  Kp,    //   38    //    P Коэффициент
  Ki,    //   39    //    I Коэффициент
  Kd,    //   40    //    D Коэффициент
  Clr,   //   41    //    очистка памяти
  Res,   //   42    //    сброс
  Tpf1,  //   43    //    режим вывода в порт плоттер
  Tpf2,  //   44    //    режим вывода в порт текст
  Lst,   //   45    //    Возможные команды
};

byte headers_am = sizeof(headers) / 2;

String prsValue = "";
String prsHeader = "";

names thisName;

enum stages { WAIT,
              HEADER,
              GOT_HEADER,
              VALUE,
              COMPLETE,
};

stages parseStage = WAIT;

int textS[] = { 5, 6, 7, 8, 9, 10, 11, 12 };

const char *myStrings[] = {
  " Стоп ",
  " Ручное управление ",
  " Влево ",
  " Вправо ",
  "Скор по двигателю",  //  17 симв
  " Скорость 2 1:",     //  14 симв
  " Скорость 3 1:",
  " Скорость 4 1:",
  " Скорость 5 1:",
  " Настройка.. ",
  "Проверьте двигатель",
  "Датчик",
  "Настройте двигатель"
};

uint8_t rFocus[8] = {
  0b00000,
  0b00000,
  0b00010,
  0b01111,
  0b00010,
  0b00000,
  0b00000,
  0b00000
};

uint8_t lFocus[8] = {
  0b00000,
  0b00000,
  0b01000,
  0b11111,
  0b01000,
  0b00000,
  0b00000,
  0b00000
};

//    текствоые строки меню
const char mainscreen1[] PROGMEM = "Настройка скоростей";
const char mainscreen2[] PROGMEM = "Настройка двигателя";
const char mainscreen3[] PROGMEM = "Режим двигателя";
const char mainscreen4[] PROGMEM = "Очистка памяти";

const char Back[] PROGMEM = "Назад";

// Это первое меню 2 уровня
const char Secmenu1_firstscreen_text1[] PROGMEM = "Обр 1 скор мин";
const char Secmenu1_firstscreen_text2[] PROGMEM = "Обр 1 скор макс";
const char Secmenu1_firstscreen_text3[] PROGMEM = "Обр 2 скор мин";
const char Secmenu1_firstscreen_text4[] PROGMEM = "Обр 2 скор макс";

const char Secmenu1_secondscreen_text1[] PROGMEM = "Обр 3 скор мин";
const char Secmenu1_secondscreen_text2[] PROGMEM = "Обр 3 скор макс";
const char Secmenu1_secondscreen_text3[] PROGMEM = "Обр 4 скор мин";
const char Secmenu1_secondscreen_text4[] PROGMEM = "Обр 4 скор макс";

const char Secmenu1_thirdscreen_text1[] PROGMEM = "Обр 5 скор мин";
const char Secmenu1_thirdscreen_text2[] PROGMEM = "Обр 5 скор макс";
const char Secmenu1_thirdscreen_text3[] PROGMEM = "Имп 2 скорости";
const char Secmenu1_thirdscreen_text4[] PROGMEM = "Имп 3 скорости";

const char Secmenu1_fourthscreen_text1[] PROGMEM = "Имп 4 скорости";
const char Secmenu1_fourthscreen_text2[] PROGMEM = "Имп 5 скорости";
const char Secmenu1_fourthscreen_text3[] PROGMEM = "Имп на 1 оборот";
const char Secmenu1_fourthscreen_text4[] PROGMEM = "Проверка имп 10 об";

const char Secmenu1_fifthscreen_text1[] PROGMEM = "Коэфф 4 скор";
const char Secmenu1_fifthscreen_text2[] PROGMEM = "Д ведущ шкива";
const char Secmenu1_fifthscreen_text3[] PROGMEM = "Д ведом шкива";

const char Secmenu1_sixthscreen_text1[] PROGMEM = "Коэфф 5 скор";
const char Secmenu1_sixthscreen_text2[] PROGMEM = "Д ведущ шкива мм";
const char Secmenu1_sixthscreen_text3[] PROGMEM = "Д ведом шкива мм";

// Это второе меню 2 уровня
const char Secmenu2_first_screen_text1[] PROGMEM = "Напр настр";
const char Secmenu2_first_screen_text2[] PROGMEM = "Режим влев / прав";
const char Secmenu2_first_screen_text3[] PROGMEM = "Пуск Стоп";
const char Secmenu2_first_screen_text4[] PROGMEM = "Период опроса мс";

const char Secmenu2_second_screen_text1[] PROGMEM = "~датчика стаб";
const char Secmenu2_second_screen_text2[] PROGMEM = "t 1 раскачки, мс";
const char Secmenu2_second_screen_text3[] PROGMEM = "dt системы в мс";
const char Secmenu2_second_screen_text4[] PROGMEM = "Точность системы ";

const char Secmenu2_third_screen_text1[] PROGMEM = "Начать настройку";
const char Secmenu2_third_screen_text2[] PROGMEM = "Коэффициент P";
const char Secmenu2_third_screen_text3[] PROGMEM = "Коэффициент I";
const char Secmenu2_third_screen_text4[] PROGMEM = "Коэффициент D";


// Это третье меню 2 уровня
const char Secmenu3_first_screen_text1[] PROGMEM = "Скорость об/м";
const char Secmenu3_first_screen_text2[] PROGMEM = "Режим лев / пр";
const char Secmenu3_first_screen_text3[] PROGMEM = "Пуск / Стоп";
const char Secmenu3_first_screen_text4[] PROGMEM = "Скор двигателя";

// Это четвертое меню 2 уровня
const char Secmenu4_first_screen_text1[] PROGMEM = "Очистка памяти";


//  ===============================    структура меню     ===============================

// Это первое меню.
LiquidMenu main_menu(lcd, main_screen, 1);
main_menu.set_focusPosition(LEFT);


// Эти строки составляют объекты главного меню.
LiquidLine Eng_speed_main_line(1, 0, mainscreen1);     //  19 симв
LiquidLine Tuning_main_line(1, 1, mainscreen2);        //  19 симв
LiquidLine Eng_mode_main_line(1, 2, mainscreen3);      //  15 симв
LiquidLine Eeprom_clear_main_line(1, 3, mainscreen4);  //  14 симв
LiquidScreen main_screen(Eng_speed_main_line, Tuning_main_line, Eng_mode_main_line, Eeprom_clear_main_line);

//  ===============================    1 меню второго уровня      ===============================

// Это второе меню 2 уровня
LiquidMenu second_menu1(lcd, second_menu1_first_screen, second_menu1_second_screen, second_menu1_third_screen, second_menu1_fourth_screen, second_menu1_fifth_screen, second_menu1_sixth_screen);
second_menu1.set_focusPosition(LEFT);

// Это первое меню 2 уровня. Параметры скоростей мин макс.
Engobmin1_second_menu1_line(1, 0, Secmenu1_firstscreen_text1, obmin[0]);  //    10 симв
Engobmax1_second_menu1_line(1, 1, Secmenu1_firstscreen_text2, obmax[0]);  //    11 симв
Engobmin2_second_menu1_line(1, 2, Secmenu1_firstscreen_text3, obmin[1]);  //    14 симв
Engobmax2_second_menu1_line(1, 3, Secmenu1_firstscreen_text4, obmax[1]);  //    15 симв
LiquidScreen second_menu1_first_screen(Engobmin1_second_menu1_line, Engobmax1_second_menu1_line, Engobmin2_second_menu1_line, Engobmax2_second_menu1_line);

Engobmin3_second_menu1_line(1, 0, Secmenu1_secondscreen_text1, obmin[2]);  //    14 симв
Engobmax3_second_menu1_line(1, 1, Secmenu1_secondscreen_text2, obmax[2]);  //    15 симв
Engobmin4_second_menu1_line(1, 2, Secmenu1_secondscreen_text3, obmin[3]);  //    14 симв
Engobmax4_second_menu1_line(1, 3, Secmenu1_secondscreen_text4, obmax[3]);  //    15 симв
LiquidScreen second_menu1_second_screen(Engobmin3_second_menu1_line, Engobmax3_second_menu1_line, Engobmin4_second_menu1_line, Engobmax4_second_menu1_line);

Engobmin5_second_menu1_line(1, 0, Secmenu1_thirdscreen_text1, obmin[3]);        //    18 симв
Engobmax5_second_menu1_line(1, 1, Secmenu1_thirdscreen_text2, obmax[3]);        //    19 симв
GearR2_second_menu1_line(1, 2, Secmenu1_thirdscreen_text2, eng_gear_ratio[0]);  //    14 симв
GearR3_second_menu1_line(1, 3, Secmenu1_thirdscreen_text3, eng_gear_ratio[1]);  //    14 симв
LiquidScreen second_menu1_third_screen(Engobmin5_second_menu1_line, Engobmax5_second_menu1_line, QImp2_second_menu1_line, QImp3_second_menu1_line);

GearR4_second_menu1_line(1, 0, Secmenu1_fourthscreen_text1, eng_gear_ratio[2]);      //    14 симв
GearR5_second_menu1_line(1, 1, Secmenu1_fourthscreen_text2, eng_gear_ratio[3]);      //    15 симв
Eng_pulse_second_menu1_line(1, 2, Secmenu1_fourthscreen_text3, eng_pulse_quantity);  //    15 симв
Eng_pulse_check_second_menu1_line(1, 3, Secmenu1_fourthscreen_text4, holl_count);    //    18 симв
LiquidScreen second_menu1_fourth_screen(QImp4_second_menu1_line, QImp5_second_menu1_line, Eng_pulse_second_menu1_line, Eng_pulse_check1_second_menu_line);

GearR4m_second_menu1_text_line(1, 0, Secmenu1_fifthscreen_text1, eng_gear_ratio[2]);           //    12 симв
Diameter_q4_pulley1_second_menu1_line(1, 1, Secmenu1_fifthscreen_text2, diameter_q4_pulley1);  //    13 симв
Diameter_q4_pulley2_second_menu1_line(1, 2, Secmenu1_fifthscreen_text3, diameter_q4_pulley2);  //    13 симв
LiquidScreen second_menu1_fifth_screen(QImp4_second_menu_text_line, Diameter_pulley1_second_menu_line, Diameter_pulley2_second_menu_line);

GearR5m_second_menu1_text_line(1, 0, Secmenu1_sixthscreen_text1, eng_gear_ratio[3]);           //    12 симв
Diameter_q5_pulley1_second_menu1_line(1, 1, Secmenu1_sixthscreen_text2, diameter_q5_pulley1);  //    13 симв
Diameter_q5_pulley2_second_menu1_line(1, 2, Secmenu1_sixthscreen_text3, diameter_q5_pulley2);  //    13 симв
Back_second_menu1_line(1, 3, Back);                                                            //    5 симв
LiquidScreen second_menu1_sixth_screen(QImp5_second_menu_text_line, Diameter_pulley1_second_menu_line, Diameter_pulley2_second_menu_line, Back_second_menu1_line);



//  ===============================    2 меню второго уровня      ===============================

// Это второе меню 2 уровня
LiquidMenu second_menu2(lcd, second_menu2_first_screen, second_menu2_second_screen, second_menu2_third_screen);
second_menu2.set_focusPosition(LEFT);

// Это второе меню 2 уровня. Настройка двигателя. Экраны.
Tundir_second_menu2_line(1, 0, Secmenu2_first_screen_text1, tuning_dir_flag);         //    19 симв
Tunsignal_second_menu2_line(1, 1, Secmenu2_first_screen_text2, tuning_signal_value);  //    18 симв
Tunstep_second_menu2_line(1, 2, Secmenu2_first_screen_text3, tuning_step_value);      //    18 симв
Tunperiod_second_menu2_line(1, 3, Secmenu2_first_screen_text4, tuning_period_value);  //    18 симв
LiquidScreen second_menu2_first_screen(Tundir_second_menu2_line, Tunsignal_second_menu2_line, Tunstep_second_menu2_line, Tunperiod_second_menu2_line);

Tunaccuracy_second_menu2_line(1, 0, Secmenu2_second_screen_text1, tuning_accuracy_value);     //    18 симв
Tunpulse_second_menu2_line(1, 1, Secmenu2_second_screen_text2, tuning_pulse_length);          //    16 симв
Tuniteration_second_menu2_line(1, 2, Secmenu2_second_screen_text3, tuning_iteration_period);  //    15 симв
Tunresultacc_second_menu2_line(1, 3, Secmenu2_second_screen_text4, tuning_result_accuracy);   //    17 симв
LiquidScreen second_menu2_second_screen(Tunaccuracy_second_menu2_line, Tunpulse_second_menu2_line, Tuniteration_second_menu2_line, Tunresultacc_second_menu2_line);

Tuning_second_menu2_line(1, 0, Secmenu2_third_screen_text1);             //    16 симв
CoeffKp_second_menu2_line(1, 1, Secmenu2_third_screen_text2, coeff_Kp);  //    13 симв
CoeffKi_second_menu2_line(1, 2, Secmenu2_third_screen_text3, coeff_Ki);  //    13 симв
CoeffKd_second_menu2_line(1, 3, Secmenu2_third_screen_text4, coeff_Kd);  //    13 симв
LiquidScreen second_menu2_third_screen(Tuning_second_menu2_line, CoeffKp_second_menu2_line, CoeffKi_second_menu2_line, CoeffKd_second_menu2_line);

Back_second_menu2_line(1, 1, Back);  //    5 симв
LiquidScreen second_menu2_fourth_screen(Back_second_menu2_line);


//  ===============================    3 меню второго уровня      ===============================

// Это третье меню 2 уровня.
LiquidMenu second_menu3(lcd, second_menu3_first_screen, second_menu3_second_screen);
second_menu3.set_focusPosition(LEFT);

// Это третье меню 2 уровня. Режим двигателя. Экраны.
Engrate_second_menu3_line(1, 0, Secmenu3_first_screen_text1, oboroti_po_regulytoru_dvigately);  //    13 симв
Engmode_second_menu3_line(1, 1, Secmenu3_first_screen_text2, Engmode);                          //    14 симв
Engstate_second_menu3_line(1, 2, Secmenu3_first_screen_text3, eng_state_flag);                  //    11 симв
Engspeedrate_second_menu3_line(1, 3, Secmenu3_first_screen_text4, eng_speed_rate);              //    14 симв
LiquidScreen second_menu3_first_screen(Engrate_second_menu3_line, Engmode_second_menu3_line, Engstate_second_menu3_line, Engspeedrate_second_menu3_line);

Back_second_menu3_line(1, 1, Back);  //    5 симв
LiquidScreen second_menu3_second_screen(Back_second_menu3_line);


//  ===============================    4 меню второго уровня      ===============================
// Это четвертое меню 2 уровня.
LiquidMenu second_menu4(lcd, second_menu4_third_screen);
second_menu4.set_focusPosition(LEFT);

// Это четвертое меню 2 уровня. Очистка Памяти. Экраны.
Eeprom_clear_second_menu4_line(1, 0, Secmenu4_first_screen_text1, Eeprom_clear_line);  //    14 симв
Back_second_menu4_line(1, 1, Back);                                                    //    5 симв
LiquidScreen second_menu4_third_screen(Eeprom_clear_second_menu4_line, Back_second_menu4_line);

// Система меню.
LiquidSystem menu_system(main_menu, second_menu1, second_menu2, second_menu3, second_menu4);
#endif

// ============================================================  Настройки  ============================================================

void setup() {
  flash.begin();

#if (Flash_prog == 0 && menu_debug == 0)
  pinMode(rele_1, OUTPUT);  //  Назначаем выходом пин A6 с реле 1
  pinMode(rele_2, OUTPUT);  //  Назначаем выходом пин A7 с реле 2

  outputString.reserve(110);

  Serial.begin(115200);
  lcd.init();  //   инициализация дисплея
  lcd.backlight();

  enc.setHoldTimeout(1200);  // установить время удержания кнопки, мс (64.. 8 000, шаг 64 мс)
  enc.setStepTimeout(500);   // установить период импульсов step, мс (32.. 4 000, шаг 32 мс)

  attachInterrupt(0, isr, RISING);  //  Прерывание по пину 2
                                    //  attachPCINT(DT);  //  Прерывание по 2 выходу энкодера
                                    //  attachPCINT(SW);  //  Прерывание по кнопке энкодера

  Watchdog.enable(RESET_MODE, WDT_PRESCALER_64);  //    сброс через 512мс

  regulator.setMode(ON_ERROR);  // режим: работа по входной ошибке ON_ERROR (0) или по изменению ON_RATE (1)


  // считываем настройки тюнинга двигателя
  EEPROM.get(14, tuning_dir_flag);
  EEPROM.get(15, tuning_signal_value);
  EEPROM.get(17, tuning_step_value);
  EEPROM.get(19, tuning_period_value);
  EEPROM.get(23, tuning_accuracy_value);
  EEPROM.get(27, tuning_pulse_length);
  EEPROM.get(31, tuning_iteration_period);
  EEPROM.get(35, tuning_result_accuracy);

  // считываем настройки минимума максимума скоростей двигателя
  EEPROM.get(41, obMin[0]);
  EEPROM.get(45, obMax[0]);
  EEPROM.get(49, obMin[1]);
  EEPROM.get(53, obMax[1]);
  EEPROM.get(57, obMin[2]);
  EEPROM.get(61, obMax[2]);
  EEPROM.get(65, obMin[3]);
  EEPROM.get(69, obMax[3]);
  EEPROM.get(73, obMin[4]);
  EEPROM.get(77, obMax[4]);

  //    считываем настройки импульсов скоростей двигателя
  EEPROM.get(81, kol_Imp_taho[1]);
  EEPROM.get(85, kol_Imp_taho[2]);
  EEPROM.get(89, kol_Imp_taho[3]);
  EEPROM.get(93, kol_Imp_taho[4]);

  //    считываем флажки настроек двигателя
  EEPROM.get(105, eng_pulse_quantity);             //    считываем количество импульсов двигателя на 1 оборот
  EEPROM.get(109, eng_speed_rate);                 //    считываем запомненную скорость двигателя
  EEPROM.get(110, eng_connect_flag);               //    флажок подключен ли двигатель
  EEPROM.get(111, eng_obstruction_detected_flag);  //    флажок было ли ранее зафиксировано препятствие
  EEPROM.get(129, tuning_complete_flag);           //    флажок был ли настроен двигатель

  //    если количестов импульсов двигателя на оборот не заполнено, то 8
  if (eng_pulse_quantity == 255 || eng_pulse_quantity == 0) eng_pulse_quantity = 8;

  // проверяем настройки тюнинга двигателя, если не заполнены
  if (tuning_signal_value != 255 && tuning_signal_value != 0 && tuning_step_value != 255 && tuning_step_value != 0 && tuning_period_value != 255 && tuning_period_value != 0 && tuning_accuracy_value != 255 && tuning_accuracy_value != 0 && tuning_pulse_length != 255 && tuning_pulse_length != 0 && tuning_iteration_period != 255 && tuning_iteration_period != 0 && tuning_result_accuracy != 255 && tuning_result_accuracy != 0) tuning_setting_incomplete_flag = false;
  else tuning_setting_incomplete_flag = true;

  // рассчитываем количество импульсов двигателя на 10 оборотов для предустановленных скоростей
  kol_Imp_taho[0] = eng_pulse_quantity * 10;

  // проверяем настройки скоростей, если не заполнены, то по двигателю.
  if (obMin[0] != 255 && obMin[0] != 0 && obMin[1] != 255 && obMin[1] != 0 && obMin[2] != 255 && obMin[2] != 0 && obMin[3] != 255 && obMin[3] != 0 && obMin[4] != 255 && obMin[4] != 0 && obMin[5] != 255 && obMin[5] != 0 && obMax[0] != 0 && obMax[1] != 255 && obMax[1] != 0 && obMax[2] != 255 && obMax[2] != 0 && obMax[3] != 255 && obMax[3] != 0 && obMax[4] != 255 && obMax[4] != 0) {
  } else eng_setting_incomplete_flag = true;


  //  ============== адреса eeprom =============================================

  //    1 байт [bool, char, byte]
  //    2 байта [int, insigned int]
  //    4 байта [long, unsigneg long, float]

  //    0, coeff_Kp                  4 байта 0..3        float
  //    4, coeff_Ki                  4 байта 4..7        float
  //    8, coeff_Kd                  4 байта 8..11       float
  //    12, eng_torm_start_power     2 байта 12..13      int
  //    14, tuning_dir_flag          1 байт  14          boolean
  //    15, tuning_signal_value      4 байта 15..18      long
  //    19, tuning_step_value        4 байта 19..22      long
  //    23, tuning_period_value      4 байта 23..26      long
  //    27, tuning_accuracy_value    4 байта 27..30      long
  //    31, tuning_pulse_length      4 байта 31..34      long
  //    35, tuning_iteration_period  4 байта 35..38      long
  //    39, tuning_result_accuracy   2 байта 39..40      int
  //    41, obMin1                   4 байта 41..44      long
  //    45, obMax1                   4 байта 45..48      long
  //    49, obMin2                   4 байта 49..42      long
  //    53, obMax2                   4 байта 53..56      long
  //    57, obMin3                   4 байта 57..60      long
  //    61, obMax3                   4 байта 61..64      long
  //    65, obMin4                   4 байта 65..68      long
  //    69, obMax4                   4 байта 69..72      long
  //    73, obMin5                   4 байта 73..76      long
  //    77, obMax5                   4 байта 77..80      long

  //    81, kol_Imp_taho[1]            4 байта 81..84      long
  //    85, kol_Imp_taho[2]            4 байта 85..88      long
  //    89, kol_Imp_taho[3]            4 байта 89..92      long
  //    93, kol_Imp_taho[4]            4 байта 93..96      long

  //    97, obMin8                   4 байта 97..100     long   //    не исп
  //    101, obMax8                  4 байта 101..104    long   //    не исп

  //    105, eng_pulse_quantity               4 байта 105..108      long
  //    109, eng_speed_rate                   2 байта 109..110      int
  //    111, eng_connect_flag                 1 байт 111            boolean
  //    112, eng_obstruction_detected_flag    1 байт 112            boolean

  //    113, diameter_q4_pulley1      4 байта 113..116      long
  //    117, diameter_q4_pulley2      4 байта 117..120      long
  //    121, diameter_q5_pulley1      4 байта 121..124      long
  //    125, diameter_q5_pulley2      4 байта 125..128      long

  //    129, tuning_complete_flag     1 байт  129           boolean
  //  ==========================================================================

  // настройки счетчика
  TCCR1B = 0;                                          //   Сброс регистра B упраления таймером 1
  TCCR1A = 0;                                          //   Сброс регистра A упраления таймером 1
  TCNT1 = 0;                                           //   Сброс счетчика
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);                //   Разрешить прерывание от сигнала на пине ICP1, разрешаем прерывание по переполению таймера
  TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS10);  //   шумоподавление 4 выборки до срабатывания, настройка прервывания по положительному перепаду, пределитель отключен 1, значение счетного регистра при срабатывании копируется в ICR1.
  ACSR = B00000111;                                    //   Захват компаратором таймер1, по переднему фронту.


  //    bitSet установка бита в 1, bitClear(BYTE, BIT) BYTE
  bitSet(PCICR, 2);  //   Разрешить прерывания порта D
  bitSet(PCMSK2, 3);
  bitSet(PCMSK2, 4);
  bitSet(PCMSK2, 5);



  // ============================================================   Прерывания  ============================================================

  ISR(TIMER1_CAPT_vect) {         //    прерывание захвата сигнала на входе ICP1
    TCNT1 = 0;                    //    обнуляем счетчик
    if (TIFR1 & (1 << TOV1)) {    //    флажок захвата таймера по входу 1 и флажок переполнения 1
      TIFR1 |= 1 << TOV1;         //    установка бита флажка срабатывания прерывания по переполнению в 1
      if (ICR1 < 100) int_tic++;  //    если значение счетного регистра менее 100 то увеличиваем переменную переполнения
    }

    tic = ((uint32_t)int_tic << 16) | ICR1;  //    подсчёт тиков копируем значение регистра счетного в переменную
    int_tic = 0;

    holl++;        //    после каждого срабатывания датчика холл + 1
    holl_count++;  //    после каждого срабатывания датчика холл + 1 для отображения в меню

    if (tic < 80000) {  //    частота расчета оборотов 200 герц,  F_CPU / 80000.
      if (millis() - interrupt_taho_timer > 5) {
        tic = median_taho.filtered(tic);
        oboroti_po_pokazaniym_taho = (16000000 / tic) / eng_pulse_quantity;  //    Высчитываем обороты по показаниям датчика в секунду
        interrupt_taho_timer = millis();
      } else {
        tic = median_taho.filtered(tic);
        oboroti_po_pokazaniym_taho = (16000000 / tic) / eng_pulse_quantity;  //    Высчитываем обороты по показаниям датчика в секунду
      }
    }
  }


  ISR(TIMER1_OVF_vect) {  //    прерывание для счёта по переполнению uint
    int_tic++;            //    считать переполнения через 65536 тактов
    if (int_tic > t) {
      tic = 0;  //    если на входе пусто более минимального времени то обнулить счётчики t = 100 переполнений таймера. (t / ( F_CPU / 65535)) = 0,4096 секунды
      int_tic = 0;
    }
  }

  /*
uint8_t attachPCINT(uint8_t pin) {
  if (pin < 8) {  // D0-D7 - PCINT2
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << pin);
    return 2;
  } else if (pin > 13) {  // A0-A5 - PCINT1
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << pin - 14);
    return 1;
  } else {  // D8-D13 - PCINT0
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << pin - 8);
    return 0;
  }
}
*/

  ISR(PCINT2_vect) {
    if (digitalRead(CLK) == HIGH) enc.tickISR();  //   Pin D3 triggered the ISR on a Rising pulse
    if (digitalRead(DT) == HIGH) enc.tickISR();   //   Pin D4 triggered the ISR on a Rising pulse
    if (digitalRead(CLK) == LOW) enc.tickISR();   //   Pin D5 triggered the ISR on a Falling pulse
  }


  void isr() {  // прерывание детектора нуля

    // вызывать в прерывании детектора нуля
    // если tickZero() - true - нужно перезапустить таймер с периодом getPeriod()

    if (dim.tickZero()) Timer2.setPeriod(dim.getPeriod());
    else Timer2.restart();
    // иначе перезапустить со старым

    zerocross_isr++;

    if (tormozenie_flag == true) {
      if (zerocross_isr == 5 or zerocross_isr == 10) multiplicity5_flag = true;

      if (zerocross_isr == 10) {
        zerocross_flag = true;  //   флажок перехода через ноль
        zerocross_isr = 0;
      }
    }
  }


  ISR(TIMER2_A) {     //   прерывание таймера
    dim.tickTimer();  //   вызвать tickTimer()
    Timer2.stop();    //   останавливаем таймер
  }

  // проверка датчика двигателя, и если нет препятствия ранее
  if (eng_connect_flag == false && eng_obstruction_detected_flag == false) {
    protection_timer = millis();
    for (engpower = 0; engpower < 40; engpower++) {
      if (millis() - protection_timer < 100) {
        engpower++;
        protection_timer = millis();
      }

    if (holl != 0 && tic < unsigned long((16 000 00 / eng_pulse_quantity)) eng_connect_flag = true;     //    (16 000 00 / eng_pulse_quantity) значения tic,  16 000 000 * 60 сек / 600 обмин * eng_pulse_quantity
    }
    EEPROM.put(110, eng_connect_flag);
  }
#endif

#if (Flash_prog == 0 && menu_debug == 0 || menu_debug == 1)

  //    ******************************      Меню      ******************************
  //  ===============================   функции первого меню первого уровня      ===============================

  Eng_speed_main_line.attach_function(enter, goto_Eng_speedrate_setting);
  Tuning_main_line.attach_function(enter, goto_Tuning);
  Eng_mode_main_line.attach_function(enter, goto_Engmode);
  Eeprom_clear_main_line.attach_function(enter, goto_Eeprom_clear);

  //  ===============================   функции первого меню второго уровня      ===============================

  Engobmin1_second_menu1_line.attach_function(increase, increase_obmin1);
  Engobmin1_second_menu1_line.attach_function(decrease, decrease_obmin1);
  Engobmin1_second_menu1_line.attach_function(changestep, step_obm);
  Engobmin1_second_menu1_line.attach_function(setfocus, focus);

  Engobmax1_second_menu1_line.attach_function(increase, increase_obmax1);
  Engobmax1_second_menu1_line.attach_function(decrease, decrease_obmax1);
  Engobmax1_second_menu1_line.attach_function(changestep, step_obm);
  Engobmax1_second_menu1_line.attach_function(setfocus, focus);

  Engobmin2_second_menu1_line.attach_function(increase, increase_obmin2);
  Engobmin2_second_menu1_line.attach_function(decrease, decrease_obmin2);
  Engobmin2_second_menu1_line.attach_function(changestep, step_obm);
  Engobmin2_second_menu1_line.attach_function(setfocus, focus);

  Engobmax2_second_menu1_line.attach_function(increase, increase_obmax2);
  Engobmax2_second_menu1_line.attach_function(decrease, decrease_obmax2);
  Engobmax2_second_menu1_line.attach_function(changestep, step_obm);
  Engobmax2_second_menu1_line.attach_function(setfocus, focus);

  Engobmin3_second_menu1_line.attach_function(increase, increase_obmin3);
  Engobmin3_second_menu1_line.attach_function(decrease, decrease_obmin3);
  Engobmin3_second_menu1_line.attach_function(changestep, step_obm);
  Engobmin3_second_menu1_line.attach_function(setfocus, focus);

  Engobmax3_second_menu1_line.attach_function(increase, increase_obmax3);
  Engobmax3_second_menu1_line.attach_function(decrease, decrease_obmax3);
  Engobmax3_second_menu1_line.attach_function(changestep, step_obm);
  Engobmax3_second_menu1_line.attach_function(setfocus, focus);

  Engobmin4_second_menu1_line.attach_function(increase, increase_obmin4);
  Engobmin4_second_menu1_line.attach_function(decrease, decrease_obmin4);
  Engobmin4_second_menu1_line.attach_function(changestep, step_obm);
  Engobmin4_second_menu1_line.attach_function(setfocus, focus);

  Engobmax4_second_menu1_line.attach_function(increase, increase_obmax4);
  Engobmax4_second_menu1_line.attach_function(decrease, decrease_obmax4);
  Engobmax4_second_menu1_line.attach_function(changestep, step_obm);
  Engobmax4_second_menu1_line.attach_function(setfocus, focus);

  Engobmin5_second_menu1_line.attach_function(increase, increase_obmin5);
  Engobmin5_second_menu1_line.attach_function(decrease, decrease_obmin5);
  Engobmin5_second_menu1_line.attach_function(changestep, step_obm);
  Engobmin5_second_menu1_line.attach_function(setfocus, focus);

  Engobmax5_second_menu1_line.attach_function(increase, increase_obmax5);
  Engobmax5_second_menu1_line.attach_function(decrease, decrease_obmax5);
  Engobmax5_second_menu1_line.attach_function(changestep, step_obm);
  Engobmax5_second_menu1_line.attach_function(setfocus, focus);

  GearR2_second_menu1_line.attach_function(increase, increase_gearR2);
  GearR2_second_menu1_line.attach_function(decrease, decrease_gearR2);
  GearR2_second_menu1_line.attach_function(setfocus, focus);

  GearR3_second_menu1_line.attach_function(increase, increase_gearR3);
  GearR3_second_menu1_line.attach_function(decrease, decrease_gearR3);
  GearR3_second_menu1_line.attach_function(setfocus, focus);

  GearR4_second_menu1_line.attach_function(increase, increase_gearR4);
  GearR4_second_menu1_line.attach_function(decrease, decrease_gearR4);
  GearR4_second_menu1_line.attach_function(changestep, step_gearR);
  GearR4_second_menu1_line.attach_function(enter, goto_gearR4);
  GearR4_second_menu1_line.attach_function(setfocus, focus);

  GearR5_second_menu1_line.attach_function(increase, increase_gearR5);
  GearR5_second_menu1_line.attach_function(decrease, decrease_gearR5);
  GearR4_second_menu1_line.attach_function(changestep, step_gearR);
  GearR5_second_menu1_line.attach_function(enter, goto_gearR5);
  GearR5_second_menu1_line.attach_function(setfocus, focus);

  Eng_pulse_second_menu1_line.attach_function(increase, increase_eng_pulse);
  Eng_pulse_second_menu1_line.attach_function(decrease, decrease_eng_pulse);
  Eng_pulse_second_menu1_line.attach_function(setfocus, focus);

  Diameter_q4_pulley1_second_menu1_line.attach_function(increase, increase_Diameter_q4_pulley1);
  Diameter_q4_pulley1_second_menu1_line.attach_function(decrease, decrease_Diameter_q4_pulley1);
  Diameter_q4_pulley1_second_menu1_line.attach_function(changestep, step_pulley);
  Diameter_q4_pulley1_second_menu1_line.attach_function(setfocus, focus);

  Diameter_q4_pulley2_second_menu1_line.attach_function(increase, increase_Diameter_q4_pulley2);
  Diameter_q4_pulley2_second_menu1_line.attach_function(decrease, decrease_Diameter_q4_pulley2);
  Diameter_q4_pulley2_second_menu1_line.attach_function(changestep, step_pulley);
  Diameter_q4_pulley2_second_menu1_line.attach_function(setfocus, focus);

  Diameter_q5_pulley1_second_menu1_line.attach_function(increase, increase_Diameter_q5_pulley1);
  Diameter_q5_pulley1_second_menu1_line.attach_function(decrease, decrease_Diameter_q5_pulley1);
  Diameter_q5_pulley1_second_menu1_line.attach_function(changestep, step_pulley);
  Diameter_q5_pulley1_second_menu1_line.attach_function(setfocus, focus);

  Diameter_q5_pulley2_second_menu1_line.attach_function(increase, increase_Diameter_q5_pulley2);
  Diameter_q5_pulley2_second_menu1_line.attach_function(decrease, decrease_Diameter_q5_pulley2);
  Diameter_q5_pulley2_second_menu1_line.attach_function(changestep, step_pulley);
  Diameter_q5_pulley2_second_menu1_line.attach_function(setfocus, focus);

  Back_second_menu1_line.attach_function(enter, goto_main_menu);


  //  ===============================   функции второго меню второго уровня      ===============================

  Tundir_second_menu2_line.attach_function(increase, increase_tuning_dir);
  Tundir_second_menu2_line.attach_function(decrease, decrease_tuning_dir);
  Tundir_second_menu2_line.attach_function(setfocus, focus);

  Tunsignal_second_menu2_line.attach_function(increase, increase_tuning_signal);
  Tunsignal_second_menu2_line.attach_function(decrease, decrease_tuning_signal);
  Tunsignal_second_menu2_line.attach_function(changestep, step_tuning_signal);
  Tunsignal_second_menu2_line.attach_function(setfocus, focus);

  Tunstep_second_menu2_line.attach_function(increase, increase_tuning_step);
  Tunstep_second_menu2_line.attach_function(decrease, decrease_tuning_step);
  Tunstep_second_menu2_line.attach_function(changestep, step_tuning_step);
  Tunstep_second_menu2_line.attach_function(setfocus, focus);

  Tunperiod_second_menu2_line.attach_function(increase, increase_tuning_period);
  Tunperiod_second_menu2_line.attach_function(decrease, decrease_tuning_period);
  Tunperiod_second_menu2_line.attach_function(changestep, step_tuning_period);
  Tunperiod_second_menu2_line.attach_function(setfocus, focus);

  Tunaccuracy_second_menu2_line.attach_function(increase, increase_tuning_accuracy);
  Tunaccuracy_second_menu2_line.attach_function(decrease, decrease_tuning_accuracy);
  Tunaccuracy_second_menu2_line.attach_function(changestep, step_tuning_accuracy);
  Tunaccuracy_second_menu2_line.attach_function(setfocus, focus);

  Tunpulse_second_menu2_line.attach_function(increase, increase_tuning_pulse_length);
  Tunpulse_second_menu2_line.attach_function(decrease, decrease_tuning_pulse_length);
  Tunpulse_second_menu2_line.attach_function(setfocus, focus);

  Tuniteration_second_menu2_line.attach_function(increase, increase_tuning_iteration_period);
  Tuniteration_second_menu2_line.attach_function(decrease, decrease_tuning_iteration_period);
  Tuniteration_second_menu2_line.attach_function(changestep, step_tuning_iteration_period);
  Tuniteration_second_menu2_line.attach_function(setfocus, focus);

  Tunresultacc_second_menu2_line.attach_function(increase, increase_tuning_result_accuracy);
  Tunresultacc_second_menu2_line.attach_function(decrease, decrease_tuning_result_accuracy);
  Tunresultacc_second_menu2_line.attach_function(changestep, step_tuning_result_accuracy);
  Tunresultacc_second_menu2_line.attach_function(setfocus, focus);

  Tuning_second_menu2_line.attach_function(enter, run_tuning);

  CoeffKp_second_menu2_line.attach_function(increase, increase_coeff_Kp);
  CoeffKp_second_menu2_line.attach_function(decrease, decrease_coeff_Kp);
  CoeffKp_second_menu2_line.attach_function(changestep, step_coeff);
  CoeffKp_second_menu2_line.attach_function(setfocus, focus);

  CoeffKi_second_menu2_line.attach_function(increase, increase_coeff_Ki);
  CoeffKi_second_menu2_line.attach_function(decrease, decrease_coeff_Ki);
  CoeffKi_second_menu2_line.attach_function(changestep, step_coeff);
  CoeffKi_second_menu2_line.attach_function(setfocus, focus);

  CoeffKd_second_menu2_line.attach_function(increase, increase_coeff_Kd);
  CoeffKd_second_menu2_line.attach_function(decrease, decrease_coeff_Kd);
  CoeffKd_second_menu2_line.attach_function(changestep, step_coeff);
  CoeffKd_second_menu2_line.attach_function(setfocus, focus);

  Back_second_menu2_line.attach_function(enter, goto_main_menu);



  //  ===============================   функции третьего меню второго уровня      ===============================

  Engrate_second_menu3_line.attach_function(increase, increase_oboroti_po_regulytoru_dvigately);
  Engrate_second_menu3_line.attach_function(decrease, decrease_oboroti_po_regulytoru_dvigately);
  Engrate_second_menu3_line.attach_function(changestep, step_oboroti_po_regulytoru_dvigately);
  Engrate_second_menu3_line.attach_function(setfocus, focus);

  Engmode_second_menu3_line.attach_function(increase, left_eng_mode);   //  функция режим влево
  Engmode_second_menu3_line.attach_function(decrease, right_eng_mode);  //  функция режим вправо
  Engmode_second_menu3_line.attach_function(setfocus, focus);           //  функция фокус строки

  Engstate_second_menu3_line(enter, eng_state);  //  функция вкл выкл двигателя
  Engstate_second_menu3_line(setfocus, focus);   //   функция фоус строки

  Engspeedrate_second_menu3_line.attach_function(increase, increase_eng_speed_rate);  //  функция увеличить скорость двигателя
  Engspeedrate_second_menu3_line.attach_function(decrease, decrease_eng_speed_rate);  //  функция уменьшить скорость двигателя
  Engspeedrate_second_menu3_line.attach_function(setfocus, focus);                    //  Выход в главное меню

  Back_second_menu3_line.attach_function(enter, goto_main_menu);  //  Выход в главное меню из 3 меню


  //  ===============================   функции четвертого меню второго уровня      ===============================

  Eeprom_clear_second_menu4_line.attach_function(enter, run_Eeprom_clear);  //  Запуск стирания памяти
  Back_second_menu4_line.attach_function(enter, goto_main_menu);            //  Выход в главное меню  из 4 меню


  menu.set_focusSymbol(Position::LEFT, lFocus);



  //    Настраиваем переменные как progmem
  Engobmin1_second_menu1_line.set_asProgmem(1);
  Engobmax1_second_menu1_line.set_asProgmem(1);
  Engobmin2_second_menu1_line.set_asProgmem(1);
  Engobmax2_second_menu1_line.set_asProgmem(1);
  Engobmin3_second_menu1_line.set_asProgmem(1);
  Engobmax3_second_menu1_line.set_asProgmem(1);
  Engobmin4_second_menu1_line.set_asProgmem(1);
  Engobmax4_second_menu1_line.set_asProgmem(1);
  Engobmin5_second_menu1_line.set_asProgmem(1);
  Engobmax5_second_menu1_line.set_asProgmem(1);
  GearR2_second_menu1_line.set_asProgmem(1);
  GearR3_second_menu1_line.set_asProgmem(1);
  GearR4_second_menu1_line.set_asProgmem(1);
  GearR5_second_menu1_line.set_asProgmem(1);
  Eng_pulse_second_menu1_line.set_asProgmem(1);
  Eng_pulse_check_second_menu1_line.set_asProgmem(1);
  GearR4m_second_menu1_text_line.set_asProgmem(1);
  Diameter_q4_pulley1_second_menu1_line.set_asProgmem(1);
  Diameter_q4_pulley2_second_menu1_line.set_asProgmem(1);
  GearR5m_second_menu1_text_line.set_asProgmem(1);
  Diameter_q5_pulley1_second_menu1_line.set_asProgmem(1);
  Diameter_q5_pulley2_second_menu1_line.set_asProgmem(1);
  Back_second_menu1_line.set_asProgmem(1);
  Tundir_second_menu2_line.set_asProgmem(1);
  Tunsignal_second_menu2_line.set_asProgmem(1);
  Tunstep_second_menu2_line.set_asProgmem(1);
  Tunperiod_second_menu2_line.set_asProgmem(1);
  Tunaccuracy_second_menu2_line.set_asProgmem(1);
  Tunpulse_second_menu2_line.set_asProgmem(1);
  Tuniteration_second_menu2_line.set_asProgmem(1);
  Tunresultacc_second_menu2_line.set_asProgmem(1);
  Tuning_second_menu2_line.set_asProgmem(1);
  CoeffKp_second_menu2_line.set_asProgmem(1);
  CoeffKi_second_menu2_line.set_asProgmem(1);
  CoeffKd_second_menu2_line.set_asProgmem(1);
  Back_second_menu2_line.set_asProgmem(1);
  Engrate_second_menu3_line.set_asProgmem(1);
  Engmode_second_menu3_line.set_asProgmem(1);
  Engstate_second_menu3_line.set_asProgmem(1);
  Engspeedrate_second_menu3_line.set_asProgmem(1);
  Back_second_menu3_line.set_asProgmem(1);
  Eeprom_clear_second_menu4_line.set_asProgmem(1);
  Back_second_menu4_line.set_asProgmem(1);
}
#endif

// ============================================================  Основной код  ============================================================


void loop() {
#if (Flash_prog == 0 && menu_debug == 0)
  enc.tick();

  relay();  //    управление реле

  if (tuning_flag == false) {  //   если не в настройке, то разрешаем обычный режим

    controls_inquiry();  //    опрашиваем панель управления

    rabota_dvigately();  //  основной цикл двигателя

  } else {

    tuning();  //    настройка двигателя
  }

  tormozenie();  //    торможение двигателя

  menu();  //   меню

  Display();  //    отображение информации

  Serial_inquiry();  //    опрос порта

  Watchdog.reset();


  boolean zero;
  long zeroctimer;

  if (zero == false) {
    zeroctimer = millis();
    zero = true;
  }

  if (millis() - zeroctimer > 100) {
    zero = false;
    zerocross_isr++;
  }

  if (tormozenie_flag == true) {
    if (zerocross_isr == 5 or zerocross_isr == 10) multiplicity5_flag = true;

    if (zerocross_isr == 10) {
      zerocross_flag = true;  //   флажок перехода через ноль
      zerocross_isr = 0;
    }
  }

#elif (menu_debug == 1)
    menu();
#endif

#if (Flash_prog == 1)
  SPIprogramming();
#endif
}

// ============================================================   ОПРОС ПАНЕЛИ  ============================================================

// =============== SETTINGS ==============
//void setButtonLevel(bool level);    // уровень кнопки: LOW - кнопка подключает GND (по умолч.), HIGH - кнопка подключает VCC
//void setHoldTimeout(int tout);      // установить время удержания кнопки, мс (64.. 8 000, шаг 64 мс)
//void setStepTimeout(int tout);      // установить период импульсов step, мс (32.. 4 000, шаг 32 мс)

//void holdEncButton(bool state);     // виртуально зажать кнопку энкодера (для срабатывания нажатых поворотов)
//void setEncReverse(bool rev);       // true - инвертировать направление энкодера (умолч. false)
//void setEncType(bool type);         // тип энкодера: EB_FULLSTEP (0) по умолч., EB_HALFSTEP (1) если энкодер делает один поворот за два щелчка


// =============== ENCODER ===============
//bool turn();            // поворот на один щелчок в любую сторону
//bool turnH();           // поворот на один щелчок в любую сторону с зажатой кнопкой
//bool fast();            // быстрый поворот на один щелчок в любую сторону
//bool right();           // поворот на один щелчок направо
//bool left();            // поворот на один щелчок налево
//bool rightH();          // поворот на один щелчок направо с зажатой кнопкой
//bool leftH();           // поворот на один щелчок налево с зажатой кнопкой
//int8_t dir();           // направление последнего поворота, 1 или -1
//int16_t counter;        // доступ к счётчику энкодера


//bool held();                      // кнопка была удержана дольше таймаута удержания. [однократно вернёт true]
//bool held(uint8_t clicks);        // кнопка была удержана с предварительным накликиванием
//bool hold();                      // кнопка удерживается дольше таймаута удержания. [возвращает true, пока удерживается]
//bool hold(uint8_t clicks);        // кнопка удерживается с предварительным накликиванием
//bool step();                      // режим импульсного удержания режим "импульсного удержания": после удержания кнопки дольше таймаута данная функция [возвращает true с периодом EB_STEP]
//bool step(uint8_t clicks);        // режим импульсного удержания с предварительным накликиванием
//bool releaseStep();               // отпущена после режима step
//bool releaseStep(uint8_t clicks); // отпущена после режима step с предварительным накликиванием
//uint8_t hasClicks();              // вернёт количество кликов, если они есть
//bool hasClicks(uint8_t num);      // проверка на наличие указанного количества кликов
//uint8_t clicks;                   // доступ к счётчику кликов


//enc.setButtonLevel(HIGH);     // уровень кнопки: LOW - кнопка подключает GND (по умолч.), HIGH - кнопка подключает VCC
//enc.setHoldTimeout(1000);     // установить время удержания кнопки, мс (до 8 000)
//enc.setStepTimeout(500);      // установить период импульсов step, мс (до 4 000)

//enc.holdEncButton(true);      // виртуально зажать кнопку энкодера (для срабатывания нажатых поворотов)
//enc.setEncReverse(true);      // true - инвертировать направление энкодера (умолч. false)
//enc.setEncType(EB_HALFSTEP);  // тип энкодера: EB_FULLSTEP (0) по умолч., EB_HALFSTEP (1) если энкодер делает один поворот за два щелчка

// =============== ЭНКОДЕР ===============
// обычный поворот
//if (enc.turn()) {
//    Serial.println("turn");

// можно ещё:
//Serial.println(enc.counter);  // вывести счётчик
//Serial.println(enc.fast());   // проверить быстрый поворот
//Serial.println(enc.dir());    // вывести направление поворота
//}

// "нажатый поворот"
//if (enc.turnH()) Serial.println("hold + turn");

//if (enc.left()) Serial.println("left");     // поворот налево
//if (enc.right()) Serial.println("right");   // поворот направо
//if (enc.leftH()) Serial.println("leftH");   // нажатый поворот налево
//if (enc.rightH()) Serial.println("rightH"); // нажатый поворот направо

// =============== КНОПКА ===============
//if (enc.press()) Serial.println("press");
//if (enc.click()) Serial.println("click");
//if (enc.release()) Serial.println("release");

//if (enc.held()) Serial.println("held");     // однократно вернёт true при удержании
//if (enc.hold()) Serial.println("hold");   // будет постоянно возвращать true после удержания
//if (enc.step()) Serial.println("step");     // импульсное удержание
//if (enc.releaseStep()) Serial.println("release step");  // отпущена после импульсного удержания

// проверка на количество кликов
//if (enc.hasClicks(1)) Serial.println("action 1 clicks");
//if (enc.hasClicks(2)) Serial.println("action 2 clicks");

// вывести количество кликов
//if (enc.hasClicks()) {
//    Serial.print("has clicks ");
//Serial.println(enc.clicks);
//}
//}







#if (Flash_prog == 0 && menu_debug == 0)
void controls_inquiry() {
  if (manual_control_flag == false || menu_flag == false || eng_obstruction_detected_flag == false) {  //   если не включен ручной режим или не в меню, и не заклинил шпиндель, то разрешаем управление
    if (enc.held()) eng_state_flag = true;                                                             //   запуск / остановка
    if (enc.held(2)) menu_flag = true;                                                                 //   2 клика и отпущена после импульсного удержания вход в меню
    if (enc.rightH()) enc_revers_right_flag = true;                                                    //   нажатый поворот вправо режим двигателя вправо
    if (enc.leftH()) enc_revers_left_flag = true;                                                      //   нажатый поворот налево режим двигателя влево
    if (enc.hold()) enc_speed_mode_flag = true;                                                        //   будет постоянно возвращать true после удержания переключить скорость
    if (enc.hasClicks(2)) enc_speed_step_flag = true;                                                  //   2 клика энкодера шаг скорости
    if (enc.left()) eng_rate_down_flag = true;                                                         //   увеличить сокрость двигателя
    if (enc.right()) eng_rate_up_flag = true;                                                          //   увеличить сокрость двигателя
  }

  if (eng_obstruction_detected_flag == true || eng_connect_flag == false) {  //  Если обнаружено препятствие, то нужно сбросить флажок, запрешаем управление и старт двигателя.
    if (enc.hold()) obstruction_multiplicity++;
    if (obstruction_multiplicity == 5) {
      eng_obstruction_detected_flag = false;
      eng_connect_flag = true;
      EEPROM.put(110, eng_connect_flag);
      EEPROM.put(111, eng_obstruction_detected_flag);
      obstruction_multiplicity = 0;
    }
  }

  if (eng_state_flag == true && tuning_complete_flag == false) {  //    запуск / остановка, и двигатель настроен
    eng_state_flag = false;
    if (was_clicked_flag == false) {                  //    если нет события нажата кнопка
      last_text = b;                                  //    запоминаем обозначение режима
      b = 0;                                          //    Во второй строке выводим "СТОП"
      last_revers_state = polozenie_tumblera_revers;  //    запоминаем прошлый режим
      polozenie_tumblera_revers = 1;                  //    режим остановлен
      was_clicked_flag = true;                        //    событие нажата кнопка
    } else {                                          //    событие нажата кнопка уже есть
      b = last_text;                                  //    Во второй строке выводим "прошлый режим"
      polozenie_tumblera_revers = last_revers_state;  //    возвращаем прошлый режим
      was_clicked_flag = false;                       //    сбрасываем событие нажата кнопка
    }
  }

  if (enc_speed_step_flag == true) {  //    если шаг скорости изменен, то изменить шаг
    enc_speed_step_flag = false;
    switch (enc_speed_step) {
      case 5: enc_speed_step += 5; break;
      case 10: enc_speed_step += 10; break;
      case 20: enc_speed_step += 5; break;
      case 25: enc_speed_step += 25; break;
      case 50: enc_speed_step += 50; break;
      case 100: enc_speed_step = 5; break;
    }
  }

  if (enc_speed_mode_flag == true) {  //    если cкорость переключена
    if (eng_speed_rate == 4) {        //    Если скорость 4
      eng_speed_rate = 0;             //    Включаем скорость 1
    } else eng_speed_rate++;          //    если нет, следующая скорость
    speed_change_timer = millis();    //    запоминаем таймер с смомента переклбчения скорости
    i = textS[eng_speed_rate];        //    выводим обозначение скорости на экран
  }

  if (enc_revers_left_flag == true) {  //    если включен режим двигателя влево
    enc_revers_left_flag = false;
    if (was_clicked_flag == true) was_clicked_flag == false;  //    если переключили по энкодеру вправо, и событие нажата кнопка уже есть, сбрасываем флажок кнопки
    b = 2;                                                    //    Во второй строке выводим "Влево"
    polozenie_tumblera_revers = 2;                            //    режим влево
  }

  if (enc_revers_right_flag == true) {  //    если включен режим двигателя вправо
    enc_revers_right_flag = false;
    if (was_clicked_flag == true) was_clicked_flag == false;  //    если переключили по энкодеру вправо, и событие нажата кнопка уже есть, сбрасываем флажок кнопки
    b = 3;                                                    //    Во второй строке выводим "Вправо"
    polozenie_tumblera_revers = 3;                            //    режим вправо
  }

  if (millis() - speed_change_timer > 3000) {
    EEPROM.put(109, eng_speed_rate);
  }

  if (obMinA == 0 || obMaxA == 0) eng_speed_setting_incomplete_flag = true;

  if (eng_speed_setting_incomplete_flag == true || enc_speed_mode_flag == true) {
    if (eng_setting_incomplete_flag == false) {                                                     //   если настройки скоростей двигателя заданы, то обычный режим
      kol_Imp_tahoA = kol_Imp_taho[eng_speed_rate];                                                 //   количество импульсов тахо на 1 скорости
      Qtr = kol_Imp_taho[eng_speed_rate] / kol_Imp_taho[0];                                         //   коэффициент передачи
      obMinA = (unsigned long)(kol_Imp_tahoA / eng_pulse_quantity) * (obMin[eng_speed_rate] / 10);  //   минимальные обороты двигателя 1 скорости
      obMaxA = (unsigned long)(kol_Imp_tahoA / eng_pulse_quantity) * (obMax[eng_speed_rate] / 10);  //   максимальные обороты двигателя 1 скорости
      i = textS[eng_speed_rate];
      oboroti_po_regulytoru_dvigately = constrain(oboroti_po_regulytoru_dvigately, obMinA, obMaxA);  //   задаем пределы скорости если переключен енкодер
      zaschita = (obMaxA / 10);                                                                      //   задаем пределы скорости для защиты
    } else {                                                                                         //   если настройки скоростей двигателя не заданы, то скорость по двигателю
      eng_speed_rate = 0;                                                                            //   1 скорость
      kol_Imp_tahoA = kol_Imp_taho[0];                                                               //   количество импульсов тахо на 1 скорости
      obMinA = (unsigned long)(kol_Imp_tahoA / eng_pulse_quantity) * (engminA / 10);                 //   минимальные обороты двигателя 1 скорости
      obMaxA = (unsigned long)(kol_Imp_tahoA / eng_pulse_quantity) * (engmaxA / 10);                 //   максимальные обороты двигателя 1 скорости
      i = textS[eng_speed_rate];
      oboroti_po_regulytoru_dvigately = constrain(oboroti_po_regulytoru_dvigately, obMinA, obMaxA);  //   задаем пределы скорости если переключен енкодер
      zaschita = (obMaxA / 10);                                                                      //   задаем пределы скорости для защиты
    }
    enc_speed_mode_flag == false;
    eng_speed_setting_incomplete_flag = false;
  }

  if (last_revers_state != polozenie_tumblera_revers && tic == 0) {  //    если тумблер реверса был переключен двигатель не вращается
    last_revers_state = polozenie_tumblera_revers;                   //    запоминаем прошлое значение реверса
    relay_flag = true;                                               //    разрешаем переключиться реле
  } else if (last_revers_state != polozenie_tumblera_revers && tic != 0) {
    capacitor_timer = millis();
    tormozenie_flag = true;  //    если тумблер реверса был переключен и двигатель вращается, то включаем торможение
  }

  if (tormozenie_flag == true && last_revers_state == polozenie_tumblera_revers && switched_flag == true && eng_to_stop_flag == true) {  //   Если торможение разрешено, и режим снова запущен, после начала торможение или остановки и реле переключилось для торможения
    relay_flag = true;                                                                                                                   //   и далее включено торможение, разрешаем включиться снова, и завершаем торможение
    tormozenie_flag = false;
    eng_speed_down_flag = false;
    switched_flag = false;
    eng_off_flag = false;
  } else if (tormozenie_flag == true && last_revers_state == polozenie_tumblera_revers && switched_flag == false && eng_to_stop_flag == false) {  //   Если торможение разрешено, и режим снова запущен, до начала сброса скорости, или после, торможение и реле переключилось для торможения
    tormozenie_flag = false;
    eng_speed_down_flag = false;
  }

  if (eng_rate_down_flag == true) {  //  Если корость переключена на меньшую, то уменьшить скорость двигателя
    oboroti_po_regulytoru_dvigately -= enc_speed_step;
    oboroti_po_regulytoru_dvigately = constrain(oboroti_po_regulytoru_dvigately, obMinA, obMaxA);
    eng_rate_down_flag = false;
  }

  if (eng_rate_up_flag == true) {  //  Если корость переключена на большую, то увеличить скорость двигателя
    oboroti_po_regulytoru_dvigately += enc_speed_step;
    oboroti_po_regulytoru_dvigately = constrain(oboroti_po_regulytoru_dvigately, obMinA, obMaxA);
    eng_rate_up_flag = false;
  }
}
#endif


// ============================================================ Меню ============================================================

void menu() {
  if (menu_flag == true) {

    //  ===============================         ===============================

    if (focused_flag = true) {                              //    Если сфокусирован на строке
      if (enc.left()) menu_system.call_function(decrease);  //    Если энкодер влево, уменьшение значения

      if (enc.right()) menu_system.call_function(increase);  //   Если энкодер вправо, увеличение значения

      if (enc.step()) menu_system.call_function(changestep);  //    Если нажат, и удеживается, изменятся шаг

      if (enc.click()) menu_system.call_function(setfocus);  //   Если кликнут, выход из фокуса

      if (enc.held()) menu_system.call_function(enter);  //   Если удержан, вызов функции

    } else {                                            //   Если не в фокусе строки
      if (enc.left()) menu_system.switch_focus(false);  //   Если удержан, вызов функции

      if (enc.right()) menu_system.switch_focus(true);  //    Если сфокусирован на строке

      if (enc.click()) menu_system.call_function(setfocus);  //   Если кликнут, вход в меню или строку
    }

    if (holl_count_flag == false) {  //    Если флажок датчика не поднят, поднимаем флажок и запоминаем таймер, для отображаения количества импульсов
      holl_count_flag = true;
      holl_count_timer = millis();
    }

    if (millis() + 8000 > holl_count_timer) holl_count = holl_count / 10;  //    Если прошло 8 секунд с момента срабатывания датчика двигателя, то считаем импульсы на 10 оборотов, отображаем в меню

    if (millis() + 13000 > holl_count_timer) {
      holl_count = 0;
      holl_count_flag = false;  //    Если прошло 13 секунд с момента срабатывания датчика двигателя
    }

    if (mem != 0) {
      switch (mem) {
        case 1: EEPROM.put(112, diameter_q4_pulley1); break;
        case 2: EEPROM.put(116, diameter_q4_pulley2); break;
        case 3: EEPROM.put(120, diameter_q5_pulley1); break;
        case 4: EEPROM.put(124, diameter_q5_pulley2); break;
        case 5: break;
      }
      menu.update();
      mem = 0;
    }
  }


  //  ===============================   функции первого меню второго уровня      ===============================

  void goto_Eng_speedrate_setting() {
    menu_system.change_menu(second_menu1);
  }

  void goto_Tuning() {
    menu_system.change_menu(second_menu2);
  }

  void goto_Engmode() {
    menu_system.change_menu(second_menu3);
  }

  void goto_Eeprom_clear() {
    menu_system.change_menu(second_menu4);
  }

  //  ===============================   функции первого меню второго уровня      ===============================

  void increase_obmin1() {
    obmin[0] += obmstep;
    obmin[0] = constrain(obmin[0], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmin1() {
    obmin[0] -= obmstep;
    obmin[0] = constrain(obmin[0], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void step_obm() {
    switch (obmstep) {
      case 5: obmstep += 5; break;
      case 10: obmstep += 10; break;
      case 20: obmstep += 5; break;
      case 25: obmstep += 25; break;
      case 50: obmstep += 50; break;
      case 100: obmstep += 100; break;
      case 200: obmstep = 5; break;
    }
  }

  void increase_obmax1() {
    obmax[0] += obmstep;
    obmax[0] = constrain(obmax[0], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmax1() {
    obmax[0] -= obmstep;
    obmax[0] = constrain(obmax[0], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmin2() {
    obmin[1] += obmstep;
    obmin[1] = constrain(obmin[1], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmin2() {
    obmin[1] -= obmstep;
    obmin[1] = constrain(obmin[1], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmax2() {
    obmax[1] += obmstep;
    obmax[1] = constrain(obmax[1], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmax2() {
    obmax[1] -= obmstep;
    obmax[1] = constrain(obmax[1], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
  }

  void increase_obmin3() {
    obmin[2] += obmstep;
    obmin[2] = constrain(obmin[2], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmin3() {
    obmin[2] -= obmstep;
    obmin[2] = constrain(obmin[2], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmax3() {
    obmax[2] += obmstep;
    obmax[2] = constrain(obmax[2], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmax3() {
    obmax[2] -= obmstep;
    obmax[2] = constrain(obmax[2], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmin4() {
    obmin[3] += obmstep;
    obmin[3] = constrain(obmin[3], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmin4() {
    obmin[3] -= obmstep;
    obmin[3] = constrain(obmin[3], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmax4() {
    obmax[3] += obmstep;
    obmax[3] = constrain(obmax[3], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmax4() {
    obmax[3] -= obmstep;
    obmax[3] = constrain(obmax[3], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmin5() {
    obmin[4] += obmstep;
    obmin[4] = constrain(obmin[4], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmin5() {
    obmin[4] -= obmstep;
    obmin[4] = constrain(obmin[4], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_obmax5() {
    obmax[4] += obmstep;
    obmax[4] = constrain(obmax[4], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void decrease_obmax5() {
    obmax[4] -= obmstep;
    obmax[4] = constrain(obmax[4], 200, 25000);  //     5, 10, 20, 25, 50, 100, 200
    mem = 5;
  }

  void increase_gearR2() {
    eng_gear_ratioI[0] += 1;
    eng_gear_ratioI[0] = constrain(eng_gear_ratioI[0], 1, 10);
    kol_Imp_taho[1] = eng_pulse_quantity * eng_gear_ratioI[0];
    EEPROM.put(81, kol_Imp_taho[1]);
    mem = 5;
  }

  void decrease_gearR2() {
    eng_gear_ratioI[0] -= 1;
    eng_gear_ratioI[0] = constrain(eng_gear_ratioI[0], 1, 10);
    kol_Imp_taho[1] = eng_pulse_quantity * eng_gear_ratioI[0];
    EEPROM.put(81, kol_Imp_taho[1]);
    mem = 5;
  }

  void increase_gearR3() {
    eng_gear_ratio[1] += 1;
    eng_gear_ratio[1] = constrain(eng_gear_ratio[1], 1, 10);
    kol_Imp_taho[2] = eng_pulse_quantity * eng_gear_ratioI[1];
    EEPROM.put(85, kol_Imp_taho[2]);
    mem = 5;
  }

  void decrease_gearR3() {
    eng_gear_ratioI[1] -= 1;
    eng_gear_ratioI[1] = constrain(eng_gear_ratioI[1], 1, 10);
    kol_Imp_taho[2] = eng_pulse_quantity * eng_gear_ratioI[1];
    EEPROM.put(85, kol_Imp_taho[2]);
    mem = 5;
  }

  void increase_gearR4() {
    eng_gear_ratioF[0] += step_gear;
    eng_gear_ratioF[0] = constrain(eng_gear_ratioF[0], 0.1, 10);
    kol_Imp_taho[3] = eng_pulse_quantity * eng_gear_ratioF[0];
    EEPROM.put(89, kol_Imp_taho[3]);
    mem = 5;
  }

  void decrease_gearR4() {
    eng_gear_ratioF[0] -= step_gear;
    eng_gear_ratioF[0] = constrain(eng_gear_ratioF[0], 0.1, 10);
    kol_Imp_taho[3] = eng_pulse_quantity * eng_gear_ratioF[0];
    EEPROM.put(89, kol_Imp_taho[3]);
    mem = 5;
  }

  void increase_gearR5() {
    eng_gear_ratioF[0] += step_gear;
    eng_gear_ratioF[0] = constrain(eng_gear_ratioF[0], 0.1, 10);
    kol_Imp_taho[4] = eng_pulse_quantity * eng_gear_ratioF[1];
    EEPROM.put(93, kol_Imp_taho[4]);
    mem = 5;
  }

  void decrease_gearR5() {
    eng_gear_ratioF[1] -= step_gear;
    eng_gear_ratioF[1] = constrain(eng_gear_ratioF[1], 0.1, 10);
    kol_Imp_taho[4] = eng_pulse_quantity * eng_gear_ratioF[1];
    EEPROM.put(93, kol_Imp_taho[4]);
    mem = 5;
  }

  void step_gearR() {
    eng_gear_ratioF[1] -= step_gear;
    eng_gear_ratioF[1] = constrain(eng_gear_ratioF[1], 0.1, 10);
    mem = 5;
  }

  void increase_eng_pulse() {
    eng_pulse_quantity += 1;
    eng_pulse_quantity = constrain(eng_pulse_quantity, 4, 20);
    mem = 5;
  }

  void decrease_eng_pulse() {
    eng_pulse_quantity -= 1;
    eng_pulse_quantity = constrain(eng_pulse_quantity, 4, 20);
    mem = 5;
  }

  void increase_Diameter_q4_pulley1() {
    diameter_q4_pulley1 += step_pulley_value;
    diameter_q4_pulley1 = constrain(diameter_q4_pulley1, 25, 100);
    mem = 1;
    gear_ratio_R4();
  }

  void decrease_Diameter_q4_pulley1() {
    diameter_q4_pulley1 -= step_pulley_value;
    diameter_q4_pulley1 = constrain(diameter_q4_pulley1, 25, 100);
    mem = 1;
    gear_ratio_R4();
  }

  void increase_Diameter_q4_pulley2() {
    diameter_q4_pulley2 += step_pulley_value;
    diameter_q4_pulley2 = constrain(diameter_q4_pulley2, 25, 300);
    mem = 2;
    gear_ratio_R4();
  }

  void decrease_Diameter_q4_pulley2() {
    diameter_q4_pulley2 -= step_pulley_value;
    diameter_q4_pulley2 = constrain(diameter_q4_pulley2, 25, 300);
    mem = 2;
    gear_ratio_R4();
  }

  void increase_Diameter_q5_pulley1() {
    diameter_q5_pulley1 += step_pulley_value;
    diameter_q5_pulley1 = constrain(diameter_q5_pulley1, 25, 100);
    mem = 3;
    gear_ratio_R5();
  }

  void decrease_Diameter_q5_pulley1() {
    diameter_q5_pulley1 -= step_pulley_value;
    diameter_q5_pulley1 = constrain(diameter_q5_pulley1, 25, 100);
    mem = 3;
    gear_ratio_R5();
  }

  void increase_Diameter_q5_pulley2() {
    diameter_q5_pulley2 += step_pulley_value;
    diameter_q5_pulley2 = constrain(diameter_q5_pulley2, 25, 300);
    mem = 4;
    gear_ratio_R5();
  }

  void decrease_Diameter_q5_pulley2() {
    diameter_q5_pulley2 -= step_pulley_value;
    diameter_q5_pulley2 = constrain(diameter_q5_pulley2, 25, 300);
    mem = 4;
    gear_ratio_R5();
  }

  void step_pulley() {
    switch (step_pulley_value) {
      case 1: step_pulley_value = 5; break;
      case 5: step_pulley_value += 5; break;
      case 10: step_pulley_value += 10; break;
      case 20: step_pulley_value += 5; break;
      case 25: step_pulley_value = 1; break;
    }
  }

  void goto_main_menu() {
    menu_system.change_menu(main_menu);
  }

  void gear_ratio_R4() {
    eng_gear_ratioF[0] = diameter_q4_pulley2 / diameter_q4_pulley1;
  }

  void gear_ratio_R5() {
    eng_gear_ratioF[1] = diameter_q5_pulley2 / diameter_q5_pulley1;
  }



  //  ===============================   функции второго меню второго уровня      ===============================

  void increase_tuning_dir() {
    tuning_dir_flag = true;
  }

  void decrease_tuning_dir() {
    tuning_dir_flag = false;
  }

  void increase_tuning_signal() {
    tuning_signal_value += step_tuning_signal_value;
    tuning_signal_value = constrain(tuning_signal_value, 2000, 20000);
    mem = 5;
  }

  void decrease_tuning_signal() {
    tuning_signal_value -= step_tuning_signal_value;
    tuning_signal_value = constrain(tuning_signal_value, 2000, 20000);
    mem = 5;
  }

  void step_tuning_signal() {
    switch (step_tuning_signal_value) {
      case 5: step_tuning_signal_value += 5; break;
      case 10: step_tuning_signal_value += 10; break;
      case 20: step_tuning_signal_value += 5; break;
      case 25: step_tuning_signal_value += 25; break;
      case 50: step_tuning_signal_value = 5; break;
    }
  }

  void increase_tuning_step() {
    tuning_step_value += step_tuning_value;
    tuning_step_value = constrain(tuning_step_value, 50, 1000);
    mem = 5;
  }

  void decrease_tuning_step() {
    tuning_step_value -= step_tuning_value;
    tuning_step_value = constrain(tuning_step_value, 50, 1000);
    mem = 5;
  }

  void step_tuning_step() {
    switch (step_tuning_value) {
      case 5: step_tuning_value += 5; break;
      case 10: step_tuning_value += 10; break;
      case 20: step_tuning_value += 5; break;
      case 25: step_tuning_value += 25; break;
      case 50: step_tuning_value = 5; break;
    }
  }

  void increase_tuning_period() {
    tuning_period_value += step_tuning_period_value;
    tuning_period_value = constrain(tuning_period_value, 5, 1000);
    mem = 5;
  }

  void decrease_tuning_period() {
    tuning_period_value -= step_tuning_period_value;
    tuning_period_value = constrain(tuning_period_value, 5, 1000);
    mem = 5;
  }

  void step_tuning_period() {
    switch (step_tuning_period_value) {
      case 5: step_tuning_period_value += 5; break;
      case 10: step_tuning_period_value += 10; break;
      case 20: step_tuning_period_value += 5; break;
      case 25: step_tuning_period_value += 25; break;
      case 50: step_tuning_period_value = 5; break;
    }
  }

  void increase_tuning_accuracy() {
    tuning_accuracy_value += 1;
    tuning_accuracy_value = constrain(tuning_accuracy_value, 1, 50);
    mem = 5;
  }

  void decrease_tuning_accuracy() {
    tuning_accuracy_value -= 1;
    tuning_accuracy_value = constrain(tuning_accuracy_value, 1, 50);
    mem = 5;
  }

  //1.1 Направление:
  //    - NORMAL: увеличение выходного сигнала увеличивает сигнал с датчика (например обогреватель, мотор)
  //- REVERSE: увеличение выходного сигнала уменьшает сигнал с датчика (например холодильник, тормоз)
  //1.2 Cигнал: базовый сигнал на управляющее устройство. Система будет ждать стабилизации по величине этого сигнала, и от него будет откладываться ступенька
  //1.3 Ступенька: величина, на которую будет изменяться сигнал в обе стороны от базового
  //1.4 Период: период опроса в ожидании стабилизации
  //1.5 Точность стабилизации: скорость изменения значения с датчика, ниже которой система будет считаться стабильной
  //1.6 Продолж. импульса: время в миллисекундах на первую раскачку
  //1.7 Период итерации: dt системы в мс, желательно должно совпадать с периодом ПИД регулятора
  // 1. Инициализация и настройка PIDtuner2 tuner;
  //tuner.setParameters(направление, начальный сигнал, конечный сигнал, период, точность, время стабилизации, период итерации)
  //Направление: - NORMAL: увеличение выходного сигнала увеличивает сигнал с датчика (например обогреватель, мотор) - REVERSE: увеличение выходного сигнала уменьшает сигнал с датчика (например холодильник, тормоз)
  //Начальный сигнал: стартовый сигнал на управляющее устройство
  //Конечный сигнал: конечный сигнал на управляющее устройство
  //Период: период опроса в ожидании стабилизации
  //Точность стабилизации: скорость изменения значения с датчика, ниже которой система будет считаться стабильной
  //Период итерации: dt системы в мс, желательно должно совпадать с периодом ПИД регулятора
  //Пример: tuner.setParameters(NORMAL, 150, 200, 1000, 1, 50); Калибруем нормальный процесс (увеличение сигнала увеличивает значение с датчика),
  //начальный сигнал 150, конечный 200, на этапе стабилизации хотим, чтобы система считалась стабильной при изменении сигнала с датчика менее, чем на 1 (условная величина датчика) за 1000 миллисекунд. Период работы всей системы - 50 мс.


  void increase_tuning_pulse_length() {
    tuning_pulse_length += 50;
    tuning_pulse_length = constrain(tuning_pulse_length, 50, 10000);
    mem = 5;
  }

  void decrease_tuning_pulse_length() {
    tuning_pulse_length -= 50;
    tuning_pulse_length = constrain(tuning_pulse_length, 50, 10000);
    mem = 5;
  }

  void increase_tuning_iteration_period() {
    tuning_iteration_period += 1;
    tuning_iteration_period = constrain(tuning_iteration_period, 1, 50);
    mem = 5;
  }

  void decrease_tuning_iteration_period() {
    tuning_iteration_period -= 1;
    tuning_iteration_period = constrain(tuning_iteration_period, 1, 50);
    mem = 5;
  }

  void increase_tuning_result_accuracy() {
    tuning_result_accuracy += 1;
    tuning_result_accuracy = constrain(tuning_result_accuracy, 80, 100);
    mem = 5;
  }

  void decrease_tuning_result_accuracy() {
    tuning_result_accuracy -= 1;
    tuning_result_accuracy = constrain(tuning_result_accuracy, 80, 100);
    mem = 5;
  }

  void run_tuning() {
    tuning_flag = true;
  }

  void increase_coeff_Kp() {
    coeff_Kp += coeff_step;
    coeff_Kp = constrain(coeff_Kp, 0.0, 100);
  }

  void decrease_coeff_Kp() {
    coeff_Kp -= coeff_step;
    coeff_Kp = constrain(coeff_Kp, 0.0, 100);
    mem = 5;
  }

  void increase_coeff_Ki() {
    coeff_Ki += coeff_step;
    coeff_Ki = constrain(coeff_Ki, 0.0, 100);
    mem = 5;
  }

  void decrease_coeff_Ki() {
    coeff_Ki -= coeff_step;
    coeff_Ki = constrain(coeff_Ki, 0.0, 100);
    mem = 5;
  }

  void increase_coeff_Kd() {
    coeff_Kd += coeff_step;
    coeff_Kd = constrain(coeff_Kd, 0.0, 100);
    mem = 5;
  }

  void decrease_coeff_Kd() {
    coeff_Kd += coeff_step;
    coeff_Kd = constrain(coeff_Kd, 0.0, 100);
    mem = 5;
  }

  void step_coeff() {
    switch (pid_coeff_step) {
      case 1: pid_coeff_step = 5; break;
      case 5: pid_coeff_step = 10; break;
      case 10: pid_coeff_step = 50; break;
      case 50: pid_coeff_step = 100; break;
      case 100: pid_coeff_step = 200; break;
      case 200: pid_coeff_step = 500; break;
    }
    coeff_step = pid_coeff_step / 1000;
  }

  void go_secmenu2_Back() {
    menu_system.change_menu(main_menu);
  }


  //  ===============================   функции третьего меню второго уровня      ===============================

  void increase_oboroti_po_regulytoru_dvigately() {
    oboroti_po_regulytoru_dvigately += enc_speed_step;
    oboroti_po_regulytoru_dvigately = constrain(oboroti_po_regulytoru_dvigately, obMinA, obMaxA);
    mem = 5;
  }

  void decrease_oboroti_po_regulytoru_dvigately() {
    oboroti_po_regulytoru_dvigately -= enc_speed_step;
    oboroti_po_regulytoru_dvigately = constrain(oboroti_po_regulytoru_dvigately, obMinA, obMaxA);
    mem = 5;
  }

  void step_oboroti_po_regulytoru_dvigately() {
    enc_speed_step_flag = true;
  }

  void left_eng_mode() {
    enc_revers_left_flag = true;
  }

  void right_eng_mode() {
    enc_revers_right_flag = true;
  }

  void eng_state() {
    eng_state_flag = true;
  }

  void increase_eng_speed_rate() {
    eng_speed_rate += 1;
    eng_speed_rate = constrain(eng_speed_rate, 0, 4);
    mem = 5;
  }

  void decrease_eng_speed_rate() {
    eng_speed_rate -= 1;
    eng_speed_rate = constrain(eng_speed_rate, 0, 4);
    mem = 5;
  }

  void go_secmenu3_Back() {
    menu_system.change_menu(main_menu);
  }

  //  ===============================   четвертого меню второго уровня      ===============================

  void run_Eeprom_clear() {
    for (spi_addr = 0; spi_addr == 105; spi_addr++) {
      EEPROM.write(spi_addr, 0);
    }
  }

  void go_secmenu4_Back() {
    menu_system.change_menu(main_menu);
  }

  void focus() {
    if (focused_flag == false) {
      focused_flag = true;
      menu.set_focusSymbol(Position::RIGHT, rFocus);
    } else {
      focused_flag = false;
      menu.set_focusSymbol(Position::RIGHT, lFocus);
      return;
    }
  }


  //  ===============================    главное меню main_menu    ===============================
  //    Eng_speedrate_setting  //    Параметры скоростей мин макс   enc.click()
  //    Tuning    //    Настройка двигателя   //    enc.click()
  //    Eng_mode    //  режим двигателя пуск стоп   //    enc.click()
  //    Eeprom_clear    //    Очистка памяти    //    enc.click()


  //  ===============================   меню Eng_speedrate_setting Параметры скоростей мин макс    ===============================
  //    Engobmin1   //    Параметр 1 скорости мин об/мин    //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmax1   //    Параметр 1 скорости макс об/мин   //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmin2   //    Параметр 2 скорости мин об/мин    //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmax2   //    Параметр 2 скорости макс об/мин   //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()


  //    ============================================
  //    Engobmin3   //    Параметр 3 скорости мин об/мин    //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmax3   //    Параметр 3 скорости макс об/мин   //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmin4   //    Параметр 4 скорости мин об/мин    //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmax4   //    Параметр 4 скорости макс об/мин   //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()


  //    ============================================
  //    Engobmin5   //    Параметр 5 скорости мин об/мин    //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engobmax5   //    Параметр 5 скорости макс об/мин   //    отображаем текущее значение, меняем, видим на экране значение, enc.held повторно выйти из фокуса строки. 5, 10, 20, 25, 50, 100, 200 Enc.step() меняем шаг после выбора пункта enc.click()
  //    QImp2   //    Отображаем "Передат 1:"значение 2 зн" 2 скор"      //    количество импульсов дописать на 10 оборотов на 2 сокрости, из передаточного числа. считаем передаточное число, отображаем число посчитанное, целое
  //    QImp3   //    Отображаем "Передат 1:"значение 2 зн" 3 скор"      //    количество импульсов дописать на 10 оборотов на 3 сокрости, из передаточного числа. считаем передаточное число, отображаем число посчитанное, целое

  //    ============================================
  //    QImp4   //    Отображаем передаточное значение 4 скорости 1:X      //    количество импульсов дописать на 10 оборотов на 4 сокрости, из передаточного числа. считаем передаточное число, отображаем число посчитанное, дробное
  //    QImp5   //    отображаем передаточное значение 5 скорости 1:X   //    количество импульсов дописать на 10 оборотов на 5 сокрости, из передаточного числа. считаем передаточное число, отображаем число посчитанное, дробное
  //    Eng_pulse  //  количество импульсов двигателя на 1 оборот   //    Импульсов на 1 оборот, отображаем число заданное, меняем, +-1 шаг
  //    Eng_pulse_check  //  проверка количества импульсов двигателя на 10 оборотов   //    отображаем при клике импульсы двигателя, считаем 8с, отображаем 5с. далее сброс.


  //  ===============================   меню QImp4 Параметры скорости 4   ===============================
  //    "Перед число 4 скор"
  //    шкив двигателя    //    диаметер, мм 4 скорость   //    отображаем значение заданное, изменяем шаг 1, 2, 5, 10, 20. Enc.step() меняем шаг после выбора пункта enc.click()
  //    шкив шпинделя    //    диаметер, мм     //    отображаем значение заданное, изменяем шаг 1, 2, 5, 10, 20. Enc.step() меняем шаг после выбора пункта enc.click()

  //  ===============================   меню QImp5 Параметры скорости 5   ===============================
  //    "Перед число 5 скор"
  //    шкив двигателя    //    диаметер, мм 5 скорость   //    отображаем значение заданное, изменяем шаг 1, 2, 5, 10, 20. Enc.step() меняем шаг после выбора пункта enc.click()
  //    шкив шпинделя    //    диаметер, мм   //    отображаем значение заданное, изменяем шаг 1, 2, 5, 10, 20. Enc.step() меняем шаг после выбора пункта enc.click()


  //  ===============================   меню Engmode режим двигателя    ===============================
  //    Engrate   //    "Скорость "значение 5зн" об/м"   //   задать скорость в оборотах в минуту отображать заданную и текущую рядом    //    отображаем значение заданное, изменяем шаг 5, 10, 20, 25, 50, 100. Enc.step() меняем шаг после выбора пункта enc.click()
  //    Engmode    //   Режим "влево" / "вправо"    //    enc.Left() и enc.Right() меняет параметр после выбора пункта enc.click()
  //    Engstate    //    Пуск Стоп   //    пуск / стоп  //   enc.held() меняет параметр
  //    Engspeedrate   //    Переключить скорость вверх    //  отображаем следующую скорость  enc.leftH() enc.rightH() меняет параметр. после выбора пункта enc.click()


  //  ===============================   меню Tuning настройка двигателя    ===============================
  //    Tundir    //    Направление настройки прямое или инверсное    //    отображаем значение заданное, изменяем прямое или инверсное
  //    Tunsignal   //    сигнал стализируемый об/мин   //    отображаем значение заданное, изменяем шаг 50, 100, 200. Enc.step() меняем шаг после выбора пункта enc.click()
  //    Tunstep   //  ступенька об/мин    //    отображаем значение заданное, изменяем шаг 1, 5, 10, 20. Enc.step() меняем шаг после выбора пункта enc.click()
  //    Tunperiod   //    период опроса в ожидании стабилизации, мс   //    отображаем значение заданное, изменяем шаг 10, 20, 25, 50, 100. Enc.step() меняем шаг после выбора пункта enc.click()


  //    ============================================
  //    Tunaccuracy   //    скорость изменения значения с датчика, ниже которой система будет считаться стабильной    //    отображаем значение заданное, изменяем шаг 1, 5, 10. Enc.step() меняем шаг после выбора пункта enc.click()
  //    Tunpulse   //    время в миллисекундах на первую раскачку, мс   //    отображаем значение заданное, изменяем шаг 10, 20, 25, 50, 100. Enc.step() меняем шаг после выбора пункта enc.click()
  //    Tuniteration    //    dt системы в мс, желательно должно совпадать с периодом ПИД регулятора    //    отображаем значение заданное, изменяем шаг 1, 2, 5. Enc.step() меняем шаг после выбора пункта enc.click()
  //    Tunresultacc    //    точность стабильности системы   //    отображаем значение заданное, изменяем шаг 1, 2, 5, 10, Enc.step() меняем шаг после выбора пункта enc.click()


  //    ============================================
  //    Настройка   //    Начать настройку    //    enc.releaseStep() запускает
  //    CoeffKp   //    Коэффициент P   //    отображаем значение заданное, изменяем 0.01, 0,05 шаг   //    отображаем значение заданное, изменяем шаг 0.01, 0.02, 0.05. Enc.step() меняем шаг после выбора пункта enc.click()
  //    CoeffKi   //    Коэффициент I   //    отображаем значение заданное, изменяем 0.01, 0,05 шаг   //    отображаем значение заданное, изменяем шаг 0.01, 0.02, 0.05. Enc.step() меняем шаг после выбора пункта enc.click()
  //    CoeffKd   //    Коэффициент D   //    отображаем значение заданное, изменяем 0.01, 0,05 шаг   //    отображаем значение заданное, изменяем шаг 0.01, 0.02, 0.05. Enc.step() меняем шаг после выбора пункта enc.click()


  //    ===============================   меню очистка памяти    ===============================
  //    очистка   //  enc.releaseStep() запускает


// ============================================================ Реле ============================================================
#if (Flash_prog == 0 && menu_debug == 0)
  void relay() {
    if (relay_flag == true && tuning_flag == false) {
      if (polozenie_tumblera_revers == 1) {  //   если реверс центр то стоп
        rew = false;
        digitalWrite(rele_1, rew);  //   переключаем реле1
        digitalWrite(rele_2, rew);  //   переключаем реле2
        relay_flag = false;
      } else if (polozenie_tumblera_revers == 2) {  //   режим влево
        rew = false;
        digitalWrite(rele_1, rew);   //   переключаем реле 1
        digitalWrite(rele_2, !rew);  //   переключаем реле 2
        relay_flag = false;
      } else if (polozenie_tumblera_revers == 3) {  //   Режим вправо
        rew = true;
        digitalWrite(rele_1, rew);   //   переключаем реле 1
        digitalWrite(rele_2, !rew);  //   переключаем реле 2
        relay_flag = false;
      }
    } else if (relay_flag == true && tuning_flag == true) {
      rew = false;
      digitalWrite(rele_1, rew);   //   переключаем реле 1
      digitalWrite(rele_2, !rew);  //   переключаем реле 2
      relay_flag = false;
    }
  }
#endif


  // ============================================================ настройка пид регулятора ============================================================

  //  Автоматический калибровщик коэффициентов ПИД регулятора, метод "реле" http://auto-controls.blogspot.com/2009/10/pid-controllers-auto-tuning-relay.html
  //Данный тюнер лучше настраивает коэффициенты для удержания величины и парирования внешних возмущений
  //Версия 1.0

  //=== Как это работает? ===
  //1. Тюнер подаёт управляющий сигнал и ждёт стабилизации значения с датчика
  //2. Тюнер изменяет сигнал на некоторую величину (ступеньку)
  //3. Ждёт заданное время, затем меняет сигнал на ту же ступеньку, но в другую сторону
  //4. Начинается раскачка системы: при прохождении значения с датчика через значение стабилизации сигнал снова переключается
  //5. Производится анализ периода раскачки и её амплитуды, на основании этих данных вычисляются рекомендуемые коэффициенты

  //=== Как пользоваться библиотекой? ===
  //1. Инициализация и настройка
  //PIDtuner tuner;
  //tuner.setParameters(направление, сигнал, ступенька, период, точность стабилизации, продолж. импульса, период итерации);

  //1.1 Направление:
  //    - NORMAL: увеличение выходного сигнала увеличивает сигнал с датчика (например обогреватель, мотор)
  //- REVERSE: увеличение выходного сигнала уменьшает сигнал с датчика (например холодильник, тормоз)
  //1.2 Cигнал: базовый сигнал на управляющее устройство. Система будет ждать стабилизации по величине этого сигнала, и от него будет откладываться ступенька
  //1.3 Ступенька: величина, на которую будет изменяться сигнал в обе стороны от базового
  //1.4 Период: период опроса в ожидании стабилизации
  //1.5 Точность стабилизации: скорость изменения значения с датчика, ниже которой система будет считаться стабильной
  //1.6 Продолж. импульса: время в миллисекундах на первую раскачку
  //1.7 Период итерации: dt системы в мс, желательно должно совпадать с периодом ПИД регулятора

  //2. Структура цикла
  //Библиотека сделана универсальной для любого датчика и управляющего устройства, цикл тюнинга организуется вот так:
  // цикл
  //tuner.setInput(значение с датчика);   // передаём текущее значение с датчика. ЖЕЛАТЕЛЬНО ФИЛЬТРОВАННОЕ
  //tuner.compute();            // тут производятся вычисления по своему таймеру
  // tuner.getOutput(); // тут можно забрать новый управляющий сигнал
  //analogWrite(pin, tuner.getOutput());  // например для ШИМ

  //3. Отладка и получение значений
  //3.1 Во время работы тюнера можно вызвать tuner.getAccuracy() - чем ближе его значение к 100, тем стабильнее на данный момент качается система и
  //тем вычисляемые коэффициенты будут более близки к идеальным
  //3.2 Для наблюдения за тюнером через Serial есть готовые методы:
  //    - tuner.debugText() выводит текстовые данные (смотри скриншот в папке docs библиотеки)
  //- tuner.debugPlot() выводит данные для построения графика через плоттер Arduino IDE (смотри скриншот в папке docs библиотеки)
  //3.3 Чтобы получить коэффициенты внутри программы (без Serial) желательно задатьт условие
  //if (tuner.getAccuracy() > 95) и при наступлении этого условия получить коэффициенты:

  //tuner.getPI_p() - p для ПИ регулятора
  //tuner.getPI_i() - i для ПИ регулятора

  //tuner.getPID_p() - p для ПИД регулятора
  //tuner.getPID_i() - i для ПИД регулятора
  //tuner.getPID_d() - d для ПИД регулятора



#if (Flash_prog == 0 && menu_debug == 0)
  void tuning() {
    if (tuning_flag == true) {
      if (tic < 400000 && in_tuning_flag == false) tormozenie_flag = true;
      i = 10;

      if (tuning_setting_incomplete_flag == false && in_tuning_flag == false && tic == 0) {
        if (tuning_dir_flag == true) tuner.setParameters(NORMAL, tuning_signal_value, tuning_step_value, tuning_period_value, tuning_accuracy_value, tuning_pulse_length, tuning_iteration_period);  //    дописать в меню
        else {
          for (spi_addr = 0; spi_addr == 10; spi_addr++) {
            outputString = "";
            flash.readStr(spi_addr, outputString);
            Serial.println(outputString);

            //"Задайте настройки тюнига"
            //"Настраиваем со стандартными параметрами"
            //"Направление нормальное"
            //"Cигнал 4000 об/мин"
            //"Ступенька 250 об/мин"
            //"Период 3000 мс"
            //"Точность стабилизации 5 об/мин"
            //"Продолжительность импульса 2000 мс"
            //"Период итерации 10 мс"
            //"Стабильность системы 95%"
          }

          tuning_signal_value = 4000;
          tuning_step_value = 250;
          tuning_period_value = 3000;
          tuning_accuracy_value = 5;
          tuning_pulse_length = 2000;
          tuning_iteration_period = 10;
          tuning_result_accuracy = 95;

          tuner.setParameters(NORMAL, tuning_signal_value, tuning_step_value, tuning_period_value, tuning_accuracy_value, tuning_pulse_length, tuning_iteration_period);
        }
        //    tuner.setParameters(NORMAL, 4000, 50, 2000, 50, 10, 50);
        //    направление, сигнал, ступенька, период, точность стабилизации, продолж. импульса, период итерации дописать в порт и в меню

        relay_flag = true;
        in_tuning_flag = true;
      }

      if (in_tuning_flag == true) {
        if (tuner.getAccuracy() < tuning_result_accuracy) {
          tuner.setInput((unsigned long)(oboroti_po_pokazaniym_taho * 60));  //    обороты в секунду фильтрованное с датчика
          tuner.compute();
          dim.write(tuner.getOutput());

          if (tuner_print_flag == true) tuner.debugPlot();
          else tuner.debugText();
        }
      }

      if (tuner.getAccuracy() > tuning_result_accuracy) {  // и при наступлении этого условия получить коэффициенты:
        regulator.Kp = tuner.getPID_p();                   // - p для ПИД регулятора
        regulator.Ki = tuner.getPID_i();                   // - i для ПИД регулятора
        regulator.Kd = tuner.getPID_d();                   // - d для ПИД регулятора

        coeff_Kp = tuner.getPID_p();
        coeff_Ki = tuner.getPID_i();
        coeff_Kd = tuner.getPID_d();

        EEPROM.put(0, coeff_Kp);
        EEPROM.put(4, coeff_Ki);
        EEPROM.put(8, coeff_Kd);

        tuning_flag == false;
        in_tuning_flag = false;
        EEPROM.put(8, coeff_Kd);
        tuning_complete_flag = true;
      }
    }
  }
#endif
  // ============================================================ Парсинг ============================================================


#if (Flash_prog == 0 && menu_debug == 0)
  void parsingSeparate() {
    if (Serial.available() > 0) {
      if (parseStage == WAIT) {
        parseStage = HEADER;
        prsHeader = "";
        prsValue = "";
      }
      if (parseStage == GOT_HEADER)
        parseStage = VALUE;
      char incoming = (char)Serial.read();
      if (incoming == divider) {
        parseStage = GOT_HEADER;
      } else if (incoming == ending) {
        parseStage = COMPLETE;
      }
      if (parseStage == HEADER)
        prsHeader += incoming;
      else if (parseStage == VALUE)
        prsValue += incoming;
      prsTimer = millis();
    }
    if (parseStage == COMPLETE) {
      for (byte i = 0; i < headers_am; i++) {
        if (prsHeader == headers[i]) thisName = i;
      }
      recievedFlag = true;
      parseStage = WAIT;
    }
    if ((millis() - prsTimer > 10) && (parseStage != WAIT)) parseStage = WAIT;  // таймаут
  }
#endif


  // ============================================================ Опрос порта ============================================================


#if (Flash_prog == 0 && menu_debug == 0)
  void Serial_inquiry() {
    parsingSeparate();

    if (recievedFlag) {
      recievedFlag = false;
      quantity_value = prsValue.toInt();
      switch (thisName) {
        case Clr:
          for (spi_addr = 0; spi_addr == 1023; spi_addr++) {
            EEPROM.write(spi_addr, 0);
          }
          spi_addr = 10;  //    "Память очищена"
          break;

        case Tun:
          spi_addr = 112;
          tuning_flag = true;
          break;

        case Mn:
          manual_control_flag = true;
          bitClear(PCMSK2, 3);
          bitClear(PCMSK2, 4);
          bitClear(PCMSK2, 5);
          spi_addr = 11;  //    Ручное управление через порт включено
          break;

        case Mf:
          manual_control_flag = false;
          bitSet(PCMSK2, 3);
          bitSet(PCMSK2, 4);
          bitSet(PCMSK2, 5);
          spi_addr = 12;  //    "Ручное управление через порт выключено"
          break;

          //    Текущие настройки тюнинга двигателя
        case Tst:
          {
            for (spi_addr = 13; spi_addr < 22, spi_addr++;) {

              if (spi_addr == 15 && tuning_dir_flag == true) continue;
              else if (spi_addr == 14 && tuning_dir_flag == false) continue;

              switch (spi_addr) {
                case 16: Serial.print(tuning_signal_value);
                case 17: Serial.print(tuning_step_value);
                case 18: Serial.print(tuning_period_value);
                case 19: Serial.print(tuning_accuracy_value);
                case 20: Serial.print(tuning_pulse_length);
                case 21: Serial.print(tuning_iteration_period);
              }

              //13   "Настройки двигателя"
              //14   "направление 1 - нормальное"
              //15   "направление 0 - инверсное"
              //16   "Cигнал, об/мин "
              //17   "Cтупенька, оборотов "
              //18   "Период стабилизации, мс "
              //19   "Точность стабилизации "
              //20   "Продолжительность импульса, мс "
              //21   "Период итерации, мс "
            }

            outputString = "";
            flash.readStr(spi_addr, outputString);
            Serial.println(outputString);
          }
          break;

        case Res:
          spi_addr = 129;
          break;

          //    текущие скорости двигателя
        case Est:
          for (spi_addr = 22; spi_addr <= 28; spi_addr++) {
            if (spi_addr > 22) {
              sprintf(printBuffer, "%s %d  %d", outputString, obMin[iter], obMax[iter]);  //    "1 скорость по двигателю мин, макс " "2 скорость мин, макс " "3 скорость мин, макс " "4 скорость мин, макс "
              Serial.println(printBuffer);
              clearprintBuffer();
            }
            outputString = "";
            flash.readStr(spi_addr, outputString);  //    "Количество импульсов на оборот "
            Serial.println(outputString);
          }
          break;

          //    Режимы двигателя
        case Lft:
          manual_control_flag = true;
          enc_revers_left_flag = true;
          if (oboroti_po_regulytoru_dvigately != 1000 && oboroti_po_regulytoru_dvigately != 0) {
          } else oboroti_po_regulytoru_dvigately = 1000;
          spi_addr = 29;  //    "Двигатель режим влево"
          break;

        case Rht:
          manual_control_flag = true;
          enc_revers_right_flag = true;
          if (oboroti_po_regulytoru_dvigately != 1000 && oboroti_po_regulytoru_dvigately != 0) {
          } else oboroti_po_regulytoru_dvigately = 1000;
          spi_addr = 30;  //    "Двигатель режим вправо"
          break;

        case On:
          manual_control_flag = true;
          polozenie_tumblera_revers = 2;
          if (oboroti_po_regulytoru_dvigately != 1000 && oboroti_po_regulytoru_dvigately != 0) {
          } else oboroti_po_regulytoru_dvigately = 1000;
          spi_addr = 31;  //    "Двигатель включен"
          break;

        case Off:
          manual_control_flag = true;
          polozenie_tumblera_revers = 1;
          b = 0;
          spi_addr = 32;  //    "Двигатель выключен"
          break;

        case Stt:
          manual_control_flag = true;
          eng_state_flag = true;
          if (oboroti_po_regulytoru_dvigately != 1000 && oboroti_po_regulytoru_dvigately != 0) {
            last_oboroti_po_regulytoru_dvigately = oboroti_po_regulytoru_dvigately;
          } else oboroti_po_regulytoru_dvigately = 1000;
          if (polozenie_tumblera_revers == 1) spi_addr = 33;  //    "Запуск двигателя"
          else spi_addr = 34;                                 //    "Остановка двигателя"
          break;

        case Rte:
          if (quantity_value < obMin[0] || quantity_value > obMax[0]) {
            sprintf(printBuffer, "%s от %d и до %d", outputString, obMin[0], obMax[0]);
            Serial.print(printBuffer);
          } else {
            oboroti_po_regulytoru_dvigately = quantity_value;
            spi_addr = 35;  //    "Скорость двигателя, об/мин "
          }
          manual_control_flag = true;
          break;

          //    скорости двигателя
        case Omi1:
          if (quantity_value < 1000 || quantity_value > 25000) {
            spi_addr = 36;  //    "От 1000 до 25000, рекомендуется 1000"
          } else {
            obMin[0] = quantity_value;
            EEPROM.put(41, obMin[0]);  //     41, obMin1                   4 байта 41..44      long
            spi_addr = 37;             //    "Минимальные обороты 1 скорости двигателя, об/мин"
          }
          break;

        case Oma1:
          if (quantity_value < 1000 || quantity_value > 25000) {
            spi_addr = 38;  //    "От 1000 до 25000, рекомендуется 10000"
          } else {
            obMax[0] = quantity_value;
            EEPROM.put(45, obMax[0]);  //    45, obMax1                   4 байта 45..48      long
            spi_addr = 39;             //    "Максимальные обороты 1 скорости двигателя, об/мин "
          }
          break;

        case Omi2:
          if (quantity_value < 500 || quantity_value > 25000) {
            spi_addr = 40;  //    "От 500 до 25000, рекомендуется 500"
          } else {
            obMin[1] = quantity_value;
            EEPROM.put(49, obMin[1]);  //    49, obMin2                   4 байта 49..42      long
            spi_addr = 41;             //    "Минимальные обороты 2 скорости двигателя, об/мин "
          }
          break;

        case Oma2:
          if (quantity_value < 500 || quantity_value > 25000) {
            spi_addr = 42;  //    "От 500 до 25000, рекомендуется 2000"
          } else {
            obMax[1] = quantity_value;
            EEPROM.put(53, obMax[1]);  //    53, obMax2                   4 байта 53..56      long
            spi_addr = 43;             //    "Максимальные обороты 2 скорости двигателя, об/мин "
          }
          break;

        case Omi3:
          if (quantity_value < 333 || quantity_value > 25000) {
            spi_addr = 44;  //    "От 333 до 25000, рекомендуется 500"
          } else {
            obMin[2] = quantity_value;
            EEPROM.put(57, obMin[2]);  //    57, obMin3                   4 байта 57..60      long
            spi_addr = 45;             //    "Минимальные обороты 3 скорости двигателя, об/мин "
          }
          break;

        case Oma3:
          if (quantity_value < 500 || quantity_value > 25000) {
            spi_addr = 46;  //    "От 500 до 25000, рекомендуется 6000"
          } else {
            obMax[2] = quantity_value;
            EEPROM.put(61, obMax[2]);  //    61, obMax3                   4 байта 61..64      long
            spi_addr = 47;             //    "Максимальные обороты 3 скорости двигателя, об/мин "
          }
          break;

        case Omi4:
          if (quantity_value < 200 || quantity_value > 25000) {
            spi_addr = 48;  //    "От 200 до 25000, рекомендуется 200"
          } else {
            obMin[3] = quantity_value;
            EEPROM.put(65, obMin[3]);  //    65, obMin4                   4 байта 65..68      long
            spi_addr = 49;             //    "Минимальные обороты 4 скорости двигателя, об/мин "
          }
          break;

        case Oma4:
          if (quantity_value < 200 || quantity_value > 25000) {
            spi_addr = 50;  //    "От 200 до 25000, рекомендуется 6000"
          } else {
            obMax[3] = quantity_value;
            EEPROM.put(69, obMax[3]);  //    69, obMax4                   4 байта 69..72      long
            spi_addr = 51;             //    "Максимальные обороты 4 скорости двигателя, об/мин "
          }
          break;

        case Omi5:
          if (quantity_value < 100 || quantity_value > 25000) {
            spi_addr = 52;  //    "От 100 до 25000, рекомендуется 100"
          } else {
            obMin[4] = quantity_value;
            EEPROM.put(73, obMin[4]);  //    73, obMin5                   4 байта 73..76      long
            spi_addr = 53;             //    "Минимальные обороты 5 скорости двигателя, об/мин "
          }
          break;

        case Oma5:
          if (quantity_value < 100 || quantity_value > 25000) {
            spi_addr = 54;  //    "От 100 до 25000, рекомендуется 100"
          } else {
            obMax[4] = quantity_value;
            EEPROM.put(77, obMax[4]);  //    77, obMax5                   4 байта 77..80      long
            spi_addr = 55;             //    "Максимальные обороты 5 скорости двигателя, об/мин "
          }
          break;

          //    тюнинг двигателя
        case Dir:
          if (quantity_value != 1 && quantity_value != 0) {
            spi_addr = 56;  //    "1 или 0, 1 нормальный, 0 инверсный"
          } else {
            if (quantity_value == 1) tuning_dir_flag = true;
            else tuning_dir_flag = false;
            EEPROM.put(14, tuning_dir_flag);             //   14, tuning_dir_flag      1 байт 14      boolean
            if (tuning_dir_flag == true) spi_addr = 57;  //    "Направления тюнера настройки пид коэффициентов 1, нормальное"
            else spi_addr = 58;                          //    "Направления тюнера настройки пид коэффициентов 0, инверсное"
          }
          break;

        case Sig:
          if (quantity_value < 1000 || quantity_value > 10000) {
            spi_addr = 59;  //    "От 1000 до 10000, рекомендуется 4000"
          } else {
            tuning_signal_value = quantity_value;
            EEPROM.put(15, tuning_signal_value);  //   15, tuning_signal_value      4 байта 15..18      long
            spi_addr = 60;                        //    "Обороты в минуту двигателя которые будем стабилизировать "
          }
          break;

        case Stp:
          if (quantity_value < 10 || quantity_value > 2000) {
            spi_addr = 61;  //    "От 10 до 2000, рекомендуется 100"
          } else {
            tuning_step_value = quantity_value;
            EEPROM.put(19, tuning_step_value);  //   19, tuning_step_value      4 байта 19..22      long
            spi_addr = 62;                      //    "Ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового, об/мин "
          }
          break;

        case Prd:
          if (quantity_value < 10 || quantity_value > 5000) {
            spi_addr = 63;  //    "От 10 до 5000, рекомендуется 1000"
          } else {
            tuning_period_value = quantity_value;
            EEPROM.put(23, tuning_period_value);  //   23, tuning_period_value      4 байта 23..26      long
            spi_addr = 64;                        //    "Период опроса в ожидании стабилизации, мс "
          }
          break;

        case Acc:
          if (quantity_value < 5 || quantity_value > 50) {
            spi_addr = 65;  //    "От 5 до 50, рекомендуется 10"
          } else {
            tuning_accuracy_value = quantity_value;
            EEPROM.put(27, tuning_accuracy_value);  //   27, tuning_accuracy_value    4 байта 27..30      long
            spi_addr = 66;                          //    "Cкорость изменения значения с датчика, ниже которой система будет считаться стабильной, об/мин "
          }
          break;

        case Pls:
          if (quantity_value < 100 || quantity_value > 5000) {
            spi_addr = 67;  //    "От 5 до 5000, рекомендуется 2000"
          } else {
            tuning_pulse_length = quantity_value;
            EEPROM.put(31, tuning_pulse_length);  //   31, tuning_pulse_length      4 байта 31..34      long
            spi_addr = 68;                        //    "Продолжительность импульса: время в миллисекундах на первую раскачку, мс "
          }
          break;

        case Itn:
          if (quantity_value < 5 || quantity_value > 50) {
            spi_addr = 69;  //    "От 5 до 50, рекомендуется 10"
          } else {
            tuning_iteration_period = quantity_value;
            EEPROM.put(35, tuning_iteration_period);  //   35, tuning_iteration_period  4 байта 35..38      long
            spi_addr = 70;                            //    "Период итерации, мс "
          }
          break;

        case Racc:
          if (quantity_value < 90 || quantity_value > 100) {
            spi_addr = 71;  //    "От 90 до 100, рекомендуется 95"
          } else {
            tuning_result_accuracy = quantity_value;
            EEPROM.put(39, tuning_result_accuracy);  //   39, tuning_result_accuracy   2 байта 39..40      int
            spi_addr = 72;                           //    "Цель стабильности системы, % "
          }
          break;

          //    параметры двигателя двигателя
        case Epl:
          if (quantity_value < 6 || quantity_value > 20) {
            spi_addr = 73;  //    "От 6 до 20, рекомендуется 8"
          } else {
            eng_pulse_quantity = quantity_value;
            EEPROM.put(39, eng_pulse_quantity);  //    105, eng_pulse_quantity      4 байта 105..108    long
            spi_addr = 74;                       //    "Количество импульсов двигателя на 1 оборот двигателя "
          }
          break;

        case Im2:
          if (quantity_value < 80 || quantity_value > 10000) {
            spi_addr = 75;  //    "От 80 до 10000, рекомендуется 1000"
          } else {
            kol_Imp_taho[1] = quantity_value;
            EEPROM.put(81, kol_Imp_taho[1]);  //    89, kol_Imp_taho[3]            4 байта 81..84      long
            spi_addr = 76;                    //    "Количество импульсов двигателя на 10 оборотов двигателя 2 скорости "
          }
          break;

        case Im3:
          if (quantity_value < 80 || quantity_value > 10000) {
            spi_addr = 77;  //    "От 80 до 10000, рекомендуется 1000"
          } else {
            kol_Imp_taho[2] = quantity_value;
            EEPROM.put(85, kol_Imp_taho[2]);  //    89, kol_Imp_taho[3]            4 байта 85..88      long
            spi_addr = 78;                    //    "Количество импульсов двигателя на 10 оборотов двигателя 3 скорости "
          }
          break;

        case Im4:
          if (quantity_value < 80 || quantity_value > 10000) {
            spi_addr = 79;  //    "От 80 до 10000, рекомендуется 1000"
          } else {
            kol_Imp_taho[3] = quantity_value;
            EEPROM.put(89, kol_Imp_taho[3]);  //    89, kol_Imp_taho[3]            4 байта 89..92      long
            spi_addr = 80;                    //    "Количество импульсов двигателя на 10 оборотов двигателя 4 скорости"
          }
          break;

        case Im5:
          if (quantity_value < 80 || quantity_value > 10000) {
            spi_addr = 81;  //    "От 80 до 10000, рекомендуется 1000"
          } else {
            kol_Imp_taho[4] = quantity_value;
            EEPROM.put(93, kol_Imp_taho[4]);  //    93, kol_Imp_taho[4]            4 байта 93..96      long
            spi_addr = 82;                    //    "Количество импульсов двигателя на 10 оборотов двигателя 5 скорости"
          }
          break;

        case Lst:
          for (spi_addr = 83; spi_addr < 128; spi_addr++) {
            flash.readStr(spi_addr, outputString);
            Serial.println(outputString);
          }
          break;


          //      "============  Команды двигателя   ============"
          //"Omi1, минимальные обороты 1 скорости, об/мин"
          //"Oma1, максимальные обороты 1 скорости, об/мин"
          //"Omi2, минимальные обороты 2 скорости, об/мин"
          //"Oma2, максимальные обороты 2 скорости, об/мин"
          //"Omi3, минимальные обороты 3 скорости, об/мин"
          //"Oma3, максимальные обороты 3 скорости, об/мин"
          //"Omi4, минимальные обороты 4 скорости, об/мин"
          //"Oma4, максимальные обороты 4 скорости, об/мин"
          //"Omi5, минимальные обороты 5 скорости, об/мин"
          //"Oma5, максимальные обороты 5 скорости, об/мин"

          //"Epl, количество импульсов на оборот двигателя"
          //"Im2, количество импульсов на 10 оборотов двигателя 2 скорости"
          //"Im3, количество импульсов на 10 оборотов двигателя 3 скорости"
          //"Im4, количество импульсов на 10 оборотов двигателя 4 скорости"
          //"Im5, количество импульсов на 10 оборотов двигателя 5 скорости"

          //"Est, текущие настройки двигателя"
          //"Rte, скорость двигателя, ручной режим"
          //"Lft, режим двигателя влево"
          //"Rht, режим двигателя вправо"
          //"On, старт двигателя"
          //"Off, остановить двигатель"
          //"Stt, пуск / стоп двигателя"
          //"Up, переключить скорость двигателя вверх, 1-5"
          //"Dn, переключить скорость двигателя вниз, 1-5"
          //"Mn, включить ручной режим"
          //"Mf, выключить ручной режим"

          //"============  Команды настройки двигателя   ============"
          //"Tst, установки настройки двигателя"
          //"Tun, начать настройку двигателя"
          //"Dir, направление настройки, 1 - нормальное, 0 - инверсное"
          //"Sig, обороты в минуту двигателя, которые будем стабилизировать"
          //"Stp, ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового, об/мин"
          //"Per, период опроса в ожидании стабилизации, мс"
          //"Acc, скорость изменения значения с датчика, ниже которой система будет считаться стабильной"
          //"Pls, продолжительность импульса: время в миллисекундах на первую раскачку, мс"
          //"Itn, период итерации, мс"
          //"Racc, целевая стабильность системы в %"
          //"Tpf1, режим вывода в порт плоттер"
          //"Tpf2, режим вывода в порт текст"
          //"Kp, коэффициент P"
          //"Ki, коэффициент I"
          //"Kd, коэффициент D"


          //"============  Команды сервисные   ============"
          //"Menu, вход меню"
          //"Clr, очистка памяти EEPROM"
          //"Res, сброс"
          //"Lst, возможные комманды"

        case Tpf1:
          spi_addr = 121;
          tuner_print_flag = true;  //    режим вывода в порт плоттер или текст
          break;

        case Tpf2:
          spi_addr = 122;
          tuner_print_flag = false;  //    режим вывода в порт плоттер или текст
          break;

        default:
          spi_addr = 129;  //  list.
          break;
      }
    }

    outputString = "";
    flash.readStr(spi_addr, outputString);
    Serial.println(outputString);

    switch (spi_addr) {
      case 28: Serial.print(eng_pulse_quantity); break;               //    адрес 28
      case 35: Serial.print(oboroti_po_regulytoru_dvigately); break;  //    адрес 35
      case 37: Serial.print(obMin[0]); break;                         //    адрес 37
      case 39: Serial.print(obMax[0]); break;                         //    адрес 39
      case 41: Serial.print(obMin[1]); break;                         //    адрес 41
      case 43: Serial.print(obMax[1]); break;                         //    адрес 43
      case 45: Serial.print(obMin[2]); break;                         //    адрес 45
      case 47: Serial.print(obMax[2]); break;                         //    адрес 47    *
      case 49: Serial.print(obMin[3]); break;                         //    адрес 49
      case 51: Serial.print(obMax[3]); break;                         //    адрес 51
      case 53: Serial.print(obMin[4]); break;                         //    адрес 53
      case 55: Serial.print(obMax[4]); break;                         //    адрес 55
      case 60: Serial.print(tuning_signal_value); break;              //    адрес 60
      case 62: Serial.print(tuning_step_value); break;                //    адрес 62
      case 64: Serial.print(tuning_period_value); break;              //    адрес 64
      case 66: Serial.print(tuning_accuracy_value); break;            //    адрес 66
      case 68: Serial.print(tuning_pulse_length); break;              //    адрес 68
      case 70: Serial.print(tuning_iteration_period); break;          //    адрес 70
      case 72: Serial.print(tuning_result_accuracy); break;           //    адрес 72
      case 74: Serial.print(eng_pulse_quantity); break;               //    адрес 74
      case 76: Serial.print(kol_Imp_taho[1]); break;                  //    адрес 76
      case 78: Serial.print(kol_Imp_taho[2]); break;                  //    адрес 78
      case 80: Serial.print(kol_Imp_taho[3]); break;                  //    адрес 80
      case 82: Serial.print(kol_Imp_taho[4]); break;                  //    адрес 82
      case 129:
        {
          Watchdog.enable(RESET_MODE, WDT_PRESCALER_2);
          while (1) {
          }
        }
        break;  //    адрес 129
    }
#endif
      /*
str.length() - текущий размер
str.clear() - очистить
str.add( [char / char* / Fchar / числа / String] ) - добавить
str += [char / char* / Fchar / числа / String] - добавить
str = str + [char / char* / Fchar / числа / String] - можно суммировать
str == [char / char* / числа / String] - сравнить
Для добавления/сравнения с mString используй str.buf

Чтение символа по индексу
str[idx]
str.buf[idx]
str.charAt(idx)

Запись символа по индексу
str[idx] = с
str.buf[idx] = с
str.setCharAt(idx, c)

Доступ к char буферу
str.buf
str.c_str()

str.toInt(from) - преобразовать в int начиная с from
str.toUint(from) - преобразовать в uint начиная с from
str.toFloat(from) - преобразовать в float начиная с from
str.startsWith(char*) - начинается с
str.substring(from, to, char* arr) - скопировать с from до to во внешний arr
str.truncate(amount) - обрезать с конца на amount
str.remove(idx, amount) - удалить (вырезать) amount символов начиная с idx
str.toLowerCase() - преобразовать буквы в нижний регистр
str.toUpperCase() - преобразовать буквы в верхний регистр
str.indexOf(char, from) - найти символ char, искать начиная с from
str.indexOf(char*, from) - найти строку char, искать начиная с from
str.split(char* str[], div) - разделить на строки по разделителю div

Парсинг пакета, в котором данные разделены разделителем div и оканчиваются символом ter
str.parseBytes(data, len, div, ter) - распарсить содержимое в массив byte длиной len
str.parseInts(data, len, div, ter) - распарсить содержимое в массив int длиной len
div и ter по умолчанию , и NULL
Например для парсинга таких пакетов: "12,34,56"
Кастомные: "12;34;56;78\n"
Парсим str.parseBytes(data, len, ';', '\n')
*/


      //float getTemp();                  // прочитать температуру с пина
      //float getTempAverage();           // прочитать усреднённую температуру с пина
      //float computeTemp(float analog);  // получить температуру из 10 бит сигнала АЦП (можно усреднённого)


      // ============================================================ ВЫВОД ЗНАЧЕНИЙ НА ЭКРАН  ============================================================


#if (Flash_prog == 0 && menu_debug == 0)
    void Display() {
      if (millis() > (Time_LCD + 500)) {
        if (tuning_flag == true && triac_breakdown_flag == false) {
          lcd.clear();
          //      clearprintBuffer();


          //    пишем режим в первой строке режим
          lcd.setCursor(0, 0);
          lcd.print(myStrings[b]);

          if (i == 5) {  //  если скорость по двигателю то скорость
            lcd.setCursor(0, 0);
            lcd.print(myStrings[i]);  //    скорость
          } else {
            lcd.setCursor(0, 0);
            lcd.print(myStrings[i]);  //    скорость
            lcd.setCursor(15, 0);
            lcd.print(Qtr);
          }


          //    пишем параметры во второй и третьей строке параметры
          lcd.setCursor(0, 1);
          outputString = "";
          flash.readStr(131, outputString);
          lcd.print(outputString);  //  режим регулятора двигателя нормальный или инверсный
          lcd.setCursor(16, 1);
          lcd.print(therm_engine.getTempAverage());  // прочитать усреднённую температуру с пина двигатель)
          lcd.setCursor(19, 1);
          lcd.print(char(223));


          lcd.setCursor(10, 1);
          outputString = "";
          flash.readStr(132, outputString);
          engine_rpm = (unsigned long)(oboroti_po_pokazaniym_taho * 60);
          lcd.print(engine_rpm);  //  об/м


          //    третья строка настройки двигателя
          lcd.setCursor(0, 2);
          lcd.print(tuning_signal_value);  //    5 символов 0..4    //    Обороты в минуту двигателя которые будем стабилизировать
          lcd.setCursor(6, 2);
          lcd.print(tuning_step_value);  //    3 символа 6..9   //    Ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового
          lcd.setCursor(11, 2);
          lcd.print(tuning_period_value);  //    4 символа 11..14   //    Период опроса в ожидании стабилизации
          lcd.setCursor(16, 2);
          lcd.print(tuning_accuracy_value);  //    2 символа 16..17   //    Cкорость изменения значения с датчика, ниже которой система будет считаться стабильной


          //    пишем процент стабильности системы в четвертой строке и остальные параметры
          lcd.setCursor(0, 3);
          lcd.print(tuning_pulse_length);  //    4 символа 0..3     //  Продолж. импульса: время в миллисекундах на первую раскачку
          lcd.setCursor(5, 3);
          lcd.print(tuning_iteration_period);  //    3 символа 5..7   //  Период итерации: dt системы в мс, желательно должно совпадать с периодом ПИД регулятора
          lcd.setCursor(9, 3);
          lcd.print(tuner.getAccuracy());  //    3 символа 9..11   //    стабильность системы в процентах
        }

        if (triac_breakdown_flag == false) {
          lcd.clear();
          if (eng_obstruction_detected_flag == false && tuning_complete_flag == true) {  //    если двигатель и датчик в норме
            engine_rpm = (unsigned long)(oboroti_po_pokazaniym_taho * 60);               //    обороты в минуту


            //    пишем скорость в первой строке
            lcd.setCursor(0, 0);
            lcd.print(myStrings[i]);
            lcd.setCursor(14, 0);
            lcd.print(Qtr, DEC);  //    пишем коэффициент


            //    пишем режим во второй строке
            lcd.setCursor(9, 1);
            lcd.print(myStrings[b]);


            //    пишем пределы скорости в третьей строке
            lcd.setCursor(0, 2);                    // от "5 знаков" до "5 знаков" обмин c 0   0..4   5..8   9..13   14..18
            lcd.print(obMin[eng_speed_rate], DEC);  //  пределы скорости минимум
            lcd.setCursor(5, 2);
            lcd.print(" до ");
            lcd.setCursor(9, 2);
            lcd.print(obMax[eng_speed_rate], DEC);  //  пределы скорости максимум
            lcd.setCursor(14, 2);
            flash.readStr(132, outputString);
            lcd.print(outputString);  //   "об/м"


            //    пишем обороты в четвертой строке
            lcd.setCursor(4, 0);
            lcd.print(engine_rpm, DEC);  //   выводим средние обороты на экран.
            flash.readStr(132, outputString);
            lcd.print(outputString);                                                            //   "об/м"
          } else if (eng_obstruction_detected_flag == true && tuning_complete_flag == false) {  //    если обнаружено препятствие
            lcd.setCursor(0, 0);
            lcd.print(myStrings[10]);  //    пишем проверьте двигатель
            lcd.setCursor(0, 1);
            lcd.print(myStrings[12]);  //   датчик
            lcd.setCursor(10, 1);
            lcd.print(holl_count);  //   отображаем число срабатываний
          } else if (eng_obstruction_detected_flag == false && tuning_complete_flag == false) {
            lcd.setCursor(0, 0);
            lcd.print(myStrings[13]);  //    пишем настройте двигатель
          }

        } else if (triac_breakdown_flag == true) {
          if (triac_breakdown_timer_flag == true) {
            lcd.clear();
            lcd.setCursor(0, 5);
            outputString = "";
            flash.readStr(133, outputString);
            lcd.print(outputString);  //    "Ошибка"
          } else if (triac_breakdown_timer_flag == false) {
            lcd.clear();
            lcd.setCursor(0, 4);
            outputString = "";
            flash.readStr(134, outputString);
            lcd.print(outputString);  //    "Пробой симистора"
          }
        }
      }


      Time_LCD = millis();
    }
#endif

    //  ============================================================  строки для вывода из памяти  ============================================================

    //    ========================    case Tuning     ========================
    //"Задайте настройки тюнига "                   //    адрес 0
    //"Настраиваем со стандартными параметрами "    //    адрес 1
    //"Направление нормальное "                     //    адрес 2
    //"Cигнал 4000 об / мин "                       //    адрес 3
    //"Ступенька 250 об / мин "                     //    адрес 4
    //"Период 3000 мс "                             //    адрес 5
    //"Точность стабилизации 5 об / мин "           //    адрес 6
    //"Продолжительность импульса 2000 мс "         //    адрес 7
    //"Период итерации 10 мс "                      //    адрес 8
    //"Стабильность системы 95 % "                  //    адрес 9

    //    ========================    case Clr     ========================
    //"Память очищена "                              //    адрес 10

    //    ========================    case Mn     ========================
    //"Ручное управление через порт включено "      //    адрес 11

    //    ========================    case Mf     ========================
    //"Ручное управление через порт выключено "      //    адрес 12

    //    ========================    case Tst     ========================

    //"Настройки двигателя "             //    адрес 13
    //"Направление 1 нормальное"         //    адрес 14
    //"Направление 0 инверсное           //    адрес 15
    //"Cигнал, об/мин "                  //    адрес 16
    //"Cтупенька, оборотов "             //    адрес 17
    //"Териод стабилизации, мс "             //    адрес 18
    //"Точность стабилизации, оборотов "     //    адрес 19
    //"Продолжительность импульса, мс "      //    адрес 20
    //"Период итерации, мс "                 //    адрес 21

    //    ========================    case Est     ========================
    //"Настройки скорости двигателя "          //    адрес 22
    //"1 скорость по двигателю мин, макс "     //    адрес 23
    //"2 скорость мин, макс "                  //    адрес 24
    //"3 скорость мин, макс "                  //    адрес 25
    //"4 скорость мин, макс "                  //    адрес 26
    //"5 скорость мин, макс "                  //    адрес 27

    //    ========================    case Epl     ========================
    //"Количество импульсов на оборот "        //    адрес 28

    //    ========================    case Lft     ========================
    //"Двигатель режим влево"                  //    адрес 29

    //    ========================    case Rht     ========================
    //"Двигатель режим вправо"                 //    адрес 30

    //    ========================    case On     ========================
    //"Двигатель включен"                      //    адрес 31

    //    ========================    case Off     ========================
    //"Двигатель выключен"                     //    адрес 32

    //    ========================    case Stt     ========================
    //"Запуск двигателя"                       //    адрес 33
    //"Остановка двигателя"                    //    адрес 34

    //    ========================    case Rte     ========================
    //"Скорость двигателя, об/мин "           //    адрес 35

    //    ========================    case Omi1     ========================
    //"От 1000 до 25000, рекомендуется 1000"                 //    адрес 36
    //"Минимальные обороты 1 скорости двигателя, об/мин "    //    адрес 37

    //    ========================    case Oma1     ========================
    //"От 1000 до 25000, рекомендуется 10000"                //    адрес 38
    //"Максимальные обороты 1 скорости двигателя, об/мин "   //    адрес 39

    //    ========================    case Omi2     ========================
    //"От 500 до 25000, рекомендуется 500"                   //    адрес 40
    //   "Минимальные обороты 2 скорости двигателя, об/мин "    //    адрес 41

    //    ========================    case Oma2     ========================
    // "От 500 до 25000, рекомендуется 2000"                  //    адрес 42
    //"Максимальные обороты 2 скорости двигателя, об/мин "   //    адрес 43

    //    ========================    case Omi3     ========================
    //"От 333 до 25000, рекомендуется 500"                   //    адрес 44
    //"Минимальные обороты 3 скорости двигателя, об/мин "    //    адрес 45

    //    ========================    case Oma3     ========================
    //"От 500 до 25000, рекомендуется 6000"                  //    адрес 46
    //"Максимальные обороты 3 скорости двигателя, об/мин "   //    адрес 47

    //    ========================    case Omi4     ========================
    //"От 200 до 25000, рекомендуется 200"                   //    адрес 48
    //"Минимальные обороты 4 скорости двигателя, об/мин "    //    адрес 49

    //    ========================    case Oma4     ========================
    //"От 200 до 25000, рекомендуется 6000"                  //    адрес 50
    //"Максимальные обороты 4 скорости двигателя, об/мин "   //    адрес 51

    //    ========================    case Omi5    ========================
    //"От 100 до 25000, рекомендуется 100"                   //    адрес 52
    //"Минимальные обороты 5 скорости двигателя, об/мин "    //    адрес 53

    //    ========================    case Oma5    ========================
    //"От 100 до 25000, рекомендуется 100"                                    //    адрес 54
    //"Максимальные обороты 5 скорости двигателя, об / мин "                  //    адрес 55

    //    ========================    case Dir    ========================
    //"1 или 0, 1 нормальный, 0 инверсный"                                   //    адрес 56
    //"Направления тюнера настройки пид коэффициентов 1, нормальное"         //    адрес 57
    //"Направления тюнера настройки пид коэффициентов 0, инверсное"          //    адрес 58

    //    ========================    case Sig    ========================
    //"От 1000 до 10000, рекомендуется 4000"                                 //    адрес 59
    //"Обороты в минуту двигателя которые будем стабилизировать "            //    адрес 60

    //    ========================    case Step    ========================
    //"От 10 до 2000, рекомендуется 100"                                                                 //    адрес 61
    //"Ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового, об/мин "     //    адрес 62

    //    ========================    case Prd    ========================
    //"От 10 до 5000, рекомендуется 1000"                                     //    адрес 63
    //"Период опроса в ожидании стабилизации, мс "                            //    адрес 64

    //    ========================    case Acc    ========================
    //"От 5 до 50, рекомендуется 10"                                                                      //    адрес 65
    //"Cкорость изменения значения с датчика, ниже которой система будет считаться стабильной, об/мин "    //    адрес 66

    //    ========================    case Pls    ========================
    //"От 5 до 5000, рекомендуется 2000"                                                 //    адрес 67
    //"Продолжительность импульса: время в миллисекундах на первую раскачку, мс "        //    адрес 68

    //    ========================    case Itn    ========================
    //"От 5 до 50, рекомендуется 10"                                              //    адрес 69
    //"Период итерации, мс "                                                      //    адрес 70

    //    ========================    case Racc    ========================
    //"От 90 до 100, рекомендуется 95"                                             //    адрес 71
    //"Цель стабильности системы, % "                                              //    адрес 72

    //    ========================    case Epl    ========================
    //"От 6 до 20, рекомендуется 8"                                              //    адрес 73
    //"Количество импульсов двигателя на 1 оборот двигателя "                    //    адрес 74

    //    ========================    case Im2    ========================
    //"От 80 до 10000, рекомендуется 1000"                                       //    адрес 75
    //"Количество импульсов двигателя на 10 оборотов двигателя 2 скорости "      //    адрес 76

    //    ========================    case Im3    ========================
    //"От 80 до 10000, рекомендуется 1000"                                       //    адрес 77
    //"Количество импульсов двигателя на 10 оборотов двигателя 3 скорости "      //    адрес 78

    //    ========================    case Im4    ========================
    //"От 80 до 10000, рекомендуется 1000"                                       //    адрес 79
    //"Количество импульсов двигателя на 10 оборотов двигателя 4 скорости "      //    адрес 80

    //    ========================    case Im5    ========================
    //"От 80 до 10000, рекомендуется 1000"                                       //    адрес 81
    //"Количество импульсов двигателя на 10 оборотов двигателя 5 скорости "      //    адрес 82

    //    ========================    case Lst    ========================
    //"============     Команды двигателя     ============"              //    адрес 83
    //"Omi1, минимальные обороты 1 скорости, об / мин "                  //    адрес 84
    //"Oma1, максимальные обороты 1 скорости, об / мин "                 //    адрес 85
    //"Omi2, минимальные обороты 2 скорости, об / мин "                  //    адрес 86
    //"Oma2, максимальные обороты 2 скорости, об / мин "                 //    адрес 87
    //"Omi3, минимальные обороты 3 скорости, об / мин "                  //    адрес 88
    //"Oma3, максимальные обороты 3 скорости, об / мин "                 //    адрес 89
    //"Omi4, минимальные обороты 4 скорости, об / мин "                  //    адрес 90
    //"Oma4, максимальные обороты 4 скорости, об / мин "                 //    адрес 91
    //"Omi5, минимальные обороты 5 скорости, об / мин "                  //    адрес 92
    //"Oma5, максимальные обороты 5 скорости, об / мин "                 //    адрес 93
    //"Epl, количество импульсов на оборот двигателя "                   //    адрес 94
    //"Im2, количество импульсов на 10 оборотов двигателя 2 скорости "      //    адрес 95
    //"Im3, количество импульсов на 10 оборотов двигателя 3 скорости "      //    адрес 96
    //"Im4, количество импульсов на 10 оборотов двигателя 4 скорости "      //    адрес 97
    //"Im5, количество импульсов на 10 оборотов двигателя 5 скорости "      //    адрес 98

    //"Est, текущие настройки двигателя "                                  //    адрес 99
    //"Rte, скорость двигателя, ручной режим "                             //    адрес 100
    //"Lft, режим двигателя влево "                                        //    адрес 101
    //"Rht, режим двигателя вправо "                                       //    адрес 102
    //"On, старт двигателя"                                                //    адрес 103
    //"Off, остановить двигатель"                                          //    адрес 104
    //"Stt, пуск / стоп двигателя"                                         //    адрес 105
    //"Up, переключить скорость двигателя вверх"                           //    адрес 106
    //"Dn, переключить скорость двигателя ввниз"                           //    адрес 107
    //"Mn, включить ручной режим"                                          //    адрес 108
    //"Mf, выключить ручной режим"                                         //    адрес 109

    //"============     Команды настройки двигателя     ============"                                         //    адрес 110
    //"Tst, установки настройки двигателя "                                                                   //    адрес 111
    //"Tun, начать настройку двигателя"                                                                       //    адрес 112
    //"Dir, направление настройки, 1 - нормальное, 0 - инверсное "                                            //    адрес 113
    //"Signal, обороты в минуту двигателя, которые будем стабилизировать "                                    //    адрес 114
    //"Stp, ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового, об/мин "       //    адрес 115
    //"Prd, период опроса в ожидании стабилизации, мс "                                                       //    адрес 116
    //"Acc, скорость изменения значения с датчика, ниже которой система будет считаться стабильной "          //    адрес 117
    //"Pls, продолжительность импульса : время в миллисекундах на первую раскачку, мс "                       //    адрес 118
    //"Itn, период итерации, мс "                                                                             //    адрес 119
    //"Racc, целевая стабильность системы в % "                                                               //    адрес 120
    //"Tpf1, режим вывода в порт плоттер"                                                                    //    адрес 121
    //"Tpf2, режим вывода в порт текст"                                                                      //    адрес 122
    //"Kp, коэффициент P"                                                                                   //    адрес 123
    //"Ki, коэффициент I"                                                                                   //    адрес 124
    //"Kd, коэффициент D"                                                                                   //    адрес 125

    //"============     Команды сервисные     ============"                 //    адрес 126
    //"Menu, вход меню"                                                     //    адрес 127
    //"Clr, очистка памяти EEPROM"                                          //    адрес 128
    //"Res, сброс"                                                          //    адрес 129

    //    ========================    case default    ========================
    //"Lst. Возможные команды"                        //    адрес 135

    //    ========================    Display    ========================
    //"Параметры "                                       //    адрес 130
    //"Нормальный "                                      //    адрес 131
    //"об/м"                                             //    адрес 132
    //"Ошибка"                                           //    адрес 133
    //"Пробой симистора"                                 //    адрес 134

    //trim()
    //outputString ="";
    //outputString += "Задайте настройки тюнига ";                  //    адрес 0
    //flash.writeStr(0, outputString);


    // ============================================================  Отладка  ============================================================

#if (Flash_prog == 1)
    void SPIprogramming() {
      if (flash_programed_flag == false) {
        for (spi_addr = 0; spi_addr <= 135; spi_addr++) {
          flash.eraseSector(spi_addr);
          outputString = "";

          switch (spi_addr) {
            case 0:
              outputString += F("Задайте настройки тюнига ");
              break;                                                                                                                                //    адрес 0
            case 1: outputString += F("Настраиваем со стандартными параметрами "); break;                                                           //    адрес 1
            case 2: outputString += F("Направление нормальное "); break;                                                                            //    адрес 2
            case 3: outputString += F("Cигнал 4000 об / мин "); break;                                                                              //    адрес 3
            case 4: outputString += F("Ступенька 250 об / мин "); break;                                                                            //    адрес 4
            case 5: outputString += F("Период 3000 мс "); break;                                                                                    //    адрес 5
            case 6: outputString += F("Точность стабилизации 5 об / мин "); break;                                                                  //    адрес 6
            case 7: outputString += F("Продолжительность импульса 2000 мс "); break;                                                                //    адрес 7
            case 8: outputString += F("Период итерации 10 мс "); break;                                                                             //    адрес 8
            case 9: outputString += F("Стабильность системы 95 % "); break;                                                                         //    адрес 9
            case 10: outputString += F("Память очищена "); break;                                                                                   //    адрес 10
            case 11: outputString += F("Ручное управление через порт включено "); break;                                                            //    адрес 11
            case 12: outputString += F("Ручное управление через порт выключено "); break;                                                           //    адрес 12
            case 13: outputString += F("Настройки двигателя "); break;                                                                              //    адрес 13
            case 14: outputString += F("Направление 1 нормальное"); break;                                                                          //    адрес 14
            case 15: outputString += F("Направление 0 инверсное"); break;                                                                           //    адрес 15
            case 16: outputString += F("Cигнал, об/мин "); break;                                                                                   //    адрес 16
            case 17: outputString += F("Cтупенька, оборотов "); break;                                                                              //    адрес 17
            case 18: outputString += F("Териод стабилизации, мс "); break;                                                                          //    адрес 18
            case 19: outputString += F("Точность стабилизации, оборотов "); break;                                                                  //    адрес 19
            case 20: outputString += F("Продолжительность импульса, мс "); break;                                                                   //    адрес 20
            case 21: outputString += F("Период итерации, мс "); break;                                                                              //    адрес 21
            case 22: outputString += F("Настройки скорости двигателя "); break;                                                                     //    адрес 22
            case 23: outputString += F("1 скорость по двигателю мин, макс "); break;                                                                //    адрес 23
            case 24: outputString += F("2 скорость мин, макс "); break;                                                                             //    адрес 24
            case 25: outputString += F("3 скорость мин, макс "); break;                                                                             //    адрес 25
            case 26: outputString += F("4 скорость мин, макс "); break;                                                                             //    адрес 26
            case 27: outputString += F("5 скорость мин, макс "); break;                                                                             //    адрес 27
            case 28: outputString += F("Количество импульсов на оборот "); break;                                                                   //    адрес 28
            case 29: outputString += F("Двигатель режим влево"); break;                                                                             //    адрес 29
            case 30: outputString += F("Двигатель режим вправо"); break;                                                                            //    адрес 30
            case 31: outputString += F("Двигатель включен"); break;                                                                                 //    адрес 31
            case 32: outputString += F("Двигатель выключен"); break;                                                                                //    адрес 32
            case 33: outputString += F("Запуск двигателя"); break;                                                                                  //    адрес 33
            case 34: outputString += F("Остановка двигателя"); break;                                                                               //    адрес 34
            case 35: outputString += F("Запуск двигателя"); break;                                                                                  //    адрес 35
            case 36: outputString += F("От 1000 до 25000, рекомендуется 1000"); break;                                                              //    адрес 36
            case 37: outputString += F("Минимальные обороты 1 скорости двигателя, об/мин "); break;                                                 //    адрес 37
            case 38: outputString += F("От 1000 до 20000, рекомендуется 10000"); break;                                                             //    адрес 38
            case 39: outputString += F("Максимальные обороты 1 скорости двигателя, об/мин "); break;                                                //    адрес 39
            case 40: outputString += F("От 500 до 25000, рекомендуется 500"); break;                                                                //    адрес 40
            case 41: outputString += F("Минимальные обороты 2 скорости двигателя, об/мин "); break;                                                 //    адрес 41
            case 42: outputString += F("От 500 до 25000, рекомендуется 2000"); break;                                                               //    адрес 42
            case 43: outputString += F("Максимальные обороты 2 скорости двигателя, об/мин "); break;                                                //    адрес 43
            case 44: outputString += F("От 333 до 25000, рекомендуется 500"); break;                                                                //    адрес 44
            case 45: outputString += F("Минимальные обороты 3 скорости двигателя, об/мин "); break;                                                 //    адрес 45
            case 46: outputString += F("От 500 до 25000, рекомендуется 6000"); break;                                                               //    адрес 46
            case 47: outputString += F("Максимальные обороты 3 скорости двигателя, об/мин "); break;                                                //    адрес 47    *
            case 48: outputString += F("От 200 до 25000, рекомендуется 200"); break;                                                                //    адрес 48
            case 49: outputString += F("Минимальные обороты 4 скорости двигателя, об/мин "); break;                                                 //    адрес 49
            case 50: outputString += F("От 200 до 25000, рекомендуется 6000"); break;                                                               //    адрес 50
            case 51: outputString += F("Максимальные обороты 4 скорости двигателя, об/мин "); break;                                                //    адрес 51
            case 52: outputString += F("От 100 до 25000, рекомендуется 100"); break;                                                                //    адрес 52
            case 53: outputString += F("Минимальные обороты 5 скорости двигателя, об/мин "); break;                                                 //    адрес 53
            case 54: outputString += F("От 100 до 25000, рекомендуется 100"); break;                                                                //    адрес 54
            case 55: outputString += F("Максимальные обороты 5 скорости двигателя, об/мин "); break;                                                //    адрес 55
            case 56: outputString += F("1 или 0, 1 нормальный, 0 инверсный"); break;                                                                //    адрес 56
            case 57: outputString += F("Направления тюнера настройки пид коэффициентов 1, нормальное"); break;                                      //    адрес 57
            case 58: outputString += F("Направления тюнера настройки пид коэффициентов 0, инверсное"); break;                                       //    адрес 58
            case 59: outputString += F("От 1000 до 10000, рекомендуется 4000"); break;                                                              //    адрес 59
            case 60: outputString += F("Обороты в минуту двигателя которые будем стабилизировать "); break;                                         //    адрес 60
            case 61: outputString += F("От 10 до 2000, рекомендуется 100"); break;                                                                  //    адрес 61
            case 62: outputString += F("Ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового, об/мин "); break;        //    адрес 62
            case 63: outputString += F("От 10 до 5000, рекомендуется 1000"); break;                                                                 //    адрес 63
            case 64: outputString += F("Период опроса в ожидании стабилизации, мс "); break;                                                        //    адрес 64
            case 65: outputString += F("От 5 до 50, рекомендуется 10"); break;                                                                      //    адрес 65
            case 66: outputString += F("Cкорость изменения значения с датчика, ниже которой система будет считаться стабильной, об/мин "); break;   //    адрес 66
            case 67: outputString += F("От 5 до 5000, рекомендуется 2000"); break;                                                                  //    адрес 67
            case 68: outputString += F("Продолжительность импульса: время в миллисекундах на первую раскачку, мс "); break;                         //    адрес 68
            case 69: outputString += F("От 5 до 50, рекомендуется 10"); break;                                                                      //    адрес 69
            case 70: outputString += F("Период итерации, мс "); break;                                                                              //    адрес 70
            case 71: outputString += F("От 90 до 100, рекомендуется 95"); break;                                                                    //    адрес 71
            case 72: outputString += F("Цель стабильности системы, % "); break;                                                                     //    адрес 72
            case 73: outputString += F("От 6 до 20, рекомендуется 8"); break;                                                                       //    адрес 73
            case 74: outputString += F("Количество импульсов двигателя на 1 оборот двигателя "); break;                                             //    адрес 74
            case 75: outputString += F("От 80 до 10000, рекомендуется 1000"); break;                                                                //    адрес 75
            case 76: outputString += F("Количество импульсов двигателя на 10 оборотов двигателя 2 скорости "); break;                               //    адрес 76
            case 77: outputString += F("От 80 до 10000, рекомендуется 1000"); break;                                                                //    адрес 77
            case 78: outputString += F("Количество импульсов двигателя на 10 оборотов двигателя 3 скорости "); break;                               //    адрес 78
            case 79: outputString += F("От 80 до 10000, рекомендуется 1000"); break;                                                                //    адрес 79
            case 80: outputString += F("Количество импульсов двигателя на 10 оборотов двигателя 4 скорости "); break;                               //    адрес 80
            case 81: outputString += F("От 80 до 10000, рекомендуется 1000"); break;                                                                //    адрес 81
            case 82: outputString += F("Количество импульсов двигателя на 10 оборот двигателя 5 скорости "); break;                                 //    адрес 82
            case 83: outputString += F("============     Команды двигателя     ============"); break;                                               //    адрес 83
            case 84: outputString += F("Omi1, минимальные обороты 1 скорости, об / мин "); break;                                                   //    адрес 84
            case 85: outputString += F("Oma1, максимальные обороты 1 скорости, об / мин "); break;                                                  //    адрес 85
            case 86: outputString += F("Omi2, минимальные обороты 2 скорости, об / мин "); break;                                                   //    адрес 86
            case 87: outputString += F("Oma2, максимальные обороты 2 скорости, об / мин "); break;                                                  //    адрес 87
            case 88: outputString += F("Omi3, минимальные обороты 3 скорости, об / мин "); break;                                                   //    адрес 88
            case 89: outputString += F("Oma3, максимальные обороты 3 скорости, об / мин "); break;                                                  //    адрес 89
            case 90: outputString += F("Omi4, минимальные обороты 4 скорости, об / мин "); break;                                                   //    адрес 90
            case 91: outputString += F("Oma4, максимальные обороты 4 скорости, об / мин "); break;                                                  //    адрес 91
            case 92: outputString += F("Omi5, минимальные обороты 5 скорости, об / мин "); break;                                                   //    адрес 92
            case 93: outputString += F("Oma5, максимальные обороты 5 скорости, об / мин "); break;                                                  //    адрес 93
            case 94: outputString += F("Epl, количество импульсов на оборот двигателя "); break;                                                    //    адрес 94
            case 95: outputString += F("Im2, количество импульсов на 10 оборотов двигателя 2 скорости "); break;                                    //    адрес 95
            case 96: outputString += F("Im3, количество импульсов на 10 оборотов двигателя 3 скорости "); break;                                    //    адрес 96
            case 97: outputString += F("Im4, количество импульсов на 10 оборотов двигателя 4 скорости "); break;                                    //    адрес 97
            case 98: outputString += F("Im5, количество импульсов на 10 оборотов двигателя 5 скорости "); break;                                    //    адрес 98
            case 99: outputString += F("Est, текущие настройки двигателя "); break;                                                                 //    адрес 99
            case 100: outputString += F("Rte, скорость двигателя, ручной режим "); break;                                                           //    адрес 100
            case 101: outputString += F("Lft, режим двигателя влево "); break;                                                                      //    адрес 101
            case 102: outputString += F("Rht, режим двигателя вправо "); break;                                                                     //    адрес 102
            case 103: outputString += F("On, старт двигателя"); break;                                                                              //    адрес 103
            case 104: outputString += F("Off, остановить двигатель"); break;                                                                        //    адрес 104
            case 105: outputString += F("Up, переключить скорость двигателя вверх"); break;                                                         //    адрес 105
            case 106: outputString += F("Dn, переключить скорость двигателя ввниз"); break;                                                         //    адрес 106
            case 107: outputString += F("Rte, скорость двигателя, ручной режим "); break;                                                           //    адрес 107
            case 108: outputString += F("Mn, включить ручной режим"); break;                                                                        //    адрес 108
            case 109: outputString += F("Mf, выключить ручной режим"); break;                                                                       //    адрес 109
            case 110: outputString += F("============     Команды настройки двигателя     ============"); break;                                    //    адрес 110
            case 111: outputString += F("Tst, установки настройки двигателя "); break;                                                              //    адрес 111
            case 112: outputString += F("Tun, начать настройку двигателя"); break;                                                                  //    адрес 112
            case 113: outputString += F("Dir, направление настройки, 1 - нормальное, 0 - инверсное "); break;                                       //    адрес 113
            case 114: outputString += F("Sig, обороты в минуту двигателя, которые будем стабилизировать "); break;                                  //    адрес 114
            case 115: outputString += F("Stp, ступенька, величина, на которую будет изменяться сигнал в обе стороны от базового, об/мин "); break;  //    адрес 115
            case 116: outputString += F("Prd, период опроса в ожидании стабилизации, мс "); break;                                                  //    адрес 116
            case 117: outputString += F("Acc, скорость изменения значения с датчика, ниже которой система будет считаться стабильной "); break;     //    адрес 117
            case 118: outputString += F("Pls, продолжительность импульса : время в миллисекундах на первую раскачку, мс "); break;                  //    адрес 118
            case 119: outputString += F("Itn, период итерации, мс "); break;                                                                        //    адрес 119
            case 120: outputString += F("Racc, целевая стабильность системы в % "); break;                                                          //    адрес 120
            case 121: outputString += F("Tpf1, режим вывода в порт плоттер"); break;                                                                //    адрес 121
            case 122: outputString += F("Tpf2  режим вывода в порт текст"); break;                                                                  //    адрес 122
            case 123: outputString += F("Kp, коэффициент P"); break;                                                                                //    адрес 123
            case 124: outputString += F("Ki, коэффициент I"); break;                                                                                //    адрес 124
            case 125: outputString += F("Kd, коэффициент D"); break;                                                                                //    адрес 125
            case 126: outputString += F("============     Команды настройки двигателя     ============"); break;                                    //    адрес 126
            case 127: outputString += F("Menu, вход меню"); break;                                                                                  //    адрес 127
            case 128: outputString += F("Clr, очистка памяти EEPROM"); break;                                                                       //    адрес 128
            case 129: outputString += F("Res, сброс"); break;                                                                                       //    адрес 129
            case 130: outputString += F("Lst. Возможные команды"); break;                                                                           //    адрес 130
            case 131: outputString += F("Параметры "); break;                                                                                       //    адрес 131
            case 132: outputString += F("Нормальный "); break;                                                                                      //    адрес 132
            case 133: outputString += F("об/м"); break;                                                                                             //    адрес 133
          case 134 outputString += F("Ошибка"); break;                                                                                           //    адрес 134
          case 135 outputString += F("Пробой симистора"); break;                                                                                 //    адрес 135
        }
        flash.writeStr(spi_addr, outputString);
      }

      if (flash_programed_flag == false) {
        Serial.println("Память запрограммирована");
        flash_programed_flag = true;
      }
    }
  }
#endif


//    дописать защиту от переключения при потере связи датчика вращения, также проверку подклчюен ли двиагатель при старте.

// ============================================================  Отладка  ============================================================
#if (Flash_prog == 0 && menu_debug == 0)
  void Debug() {
    Serial.println('*');
  }
#endif

#if (Flash_prog == 0 && menu_debug == 0)
  void clearprintBuffer() {
    for (uint8_t i = 0; i < 128; i++) {
      printBuffer[i] = 0;
    }
  }
#endif


              // ============================================================  ТОРМОЖЕНИЕ  ============================================================


#if (Flash_prog == 0 && menu_debug == 0)
  void tormozenie() {
    if (tormozenie_flag == true) {

      if (eng_speed_down_flag == false) {
        engpower = 0;
        Timer2.disableISR();
      }


      if (eng_speed_down_flag == false && tic < 1000) eng_speed_down_flag = true;  //    ждём пока двигатель не достигнет скорости пригодной для торможения, флажок, что двигатель сбросил скорость для торможения вкл

      if (switched_flag == false && eng_speed_down_flag == true) {
        if (millis() > capacitor_timer + 100) {  //    задержка чтоб разрядить конденсатор
          rew = !rew;                            //    для переключения реле для торможения
          digitalWrite(rele_1, rew);             //    переключаем реле1
          digitalWrite(rele_2, !rew);            //    переключаем реле2
        }
        switched_flag = true;     //    флажок реле переключено
        relay_timer = millis();   //    запоминаем время для переключение реле
        eng_to_stop_flag = true;  //    разрешаем торможение
      }

      if (eng_to_stop_flag == true) {       //    запускаем цикл торможения
        if (millis() > relay_timer + 50) {  //    задержка для срабатывания реле.
          if (tic < 400000) {
            if (bitRead(TIMSK2, 1) != 1) Timer2.enableISR();
          }

          if (multiplicity5_flag == true) {  //    если прошло 5 переходов через ноль, то увеличиваем торможение на единицу
            engpower++;
            engpower = constrain(engpower, eng_torm_start_power, eng_torm_full_power);  //    ограничиваем мощность при торможении
            dim.write(engpower);
            multiplicity5_flag = false;
          }
        }

        if (tic > 400000) {  //    если двигатель сбросил скорость, отключаем двигатель
          eng_to_stop_flag = false;
          eng_off_flag = true;
          engpower = 0;
          Timer2.disableISR();
        }
      }

      if (eng_off_flag == true && tic == 0) {  //    если двигатель полностью остановлен, разрешаем завершить торможение
        tormozenie_flag = false;
        eng_speed_down_flag = false;
        switched_flag = false;
        eng_off_flag = false;
      }
    }
  }
#endif


              // ============================================================  Цикл работы двигателя  ============================================================

#if (Flash_prog == 0 && menu_debug == 0)
  void rabota_dvigately() {
    if (triac_breakdown_flag == false || tormozenie_flag == false || eng_obstruction_detected_flag == false) {
      if (bitRead(TIMSK2, 1) != 1) Timer2.enableISR();

      //    if (holl == 0) {
      //    dim.write(1);
      //    eng_start_timer = millis();
      //    if (eng_start_timer - millis() > 100) {
      //        if (holl == 0) engpower++;
      //    }
      //    }

      if (holl >= 1) {  //   если сработал датчик

        if (eng_protection_timer_flag == false) {
          if ((unsigned long)(oboroti_po_pokazaniym_taho * 60) > (unsigned long)(oboroti_po_regulytoru_dvigately + zaschita)) {  //    если обороты превышают нужные + защита (10% от скорости двигателя )
            protection_timer = millis();
            oboroti_po_pokazaniym_taho_temp = oboroti_po_pokazaniym_taho;  //   запоминаем обороты двигателя
            eng_protection_timer_flag = true;
          }
        }

        if (obstruction_timer_flag == false) {  //  Если флаг таймера проверки препятствия снят, то запоминаем таймер, и поднимаем флажок
          obstruction_timer = millis();
          obstruction_timer_flag = true;
        } else if (millis() > obstruction_timer + 200) {  //  Если прошло от таймера проверки препятствия 200 мс, запоминаем обороты
          oboroti_po_pokazaniym_taho_obstruction = oboroti_po_pokazaniym_taho;
          obstruction_timer_flag = false;  // снимаем флажок
        }

        if (eng_obstruction_check_timer_flag == false) {  //  Если флаг таймера проверки на препятствие снят, то запоминаем таймер, и поднимаем флажок
          obstruction_check_timer = millis();
          eng_obstruction_check_timer_flag = true;
        } else if (millis() > obstruction_check_timer + 100) eng_obstruction_check_timer_flag == false;  //  Если прошло 100 мс снимаем флажок проверки на препятствие

        if (oboroti_po_pokazaniym_taho < (unsigned long)(oboroti_po_pokazaniym_taho_obstruction / 3) && (polozenie_tumblera_revers == 2 || polozenie_tumblera_revers == 3)) {  //    Если замечено препятствие и скорость снижения оборотов высокая, и режим двигателя влево или вправо, то включаем флаг обнаружено препятствие, отключаем двигатель
          engpower = 0;
          eng_obstruction_detected_flag = true;  //   Поднимаем флажок обнаружено препятствие
          EEPROM.put(111, eng_obstruction_detected_flag);
        }

        if (millis() > protection_timer + 700) {
          if (oboroti_po_pokazaniym_taho_temp < oboroti_po_pokazaniym_taho) {  //   обороты по регулятору больше оборотов двигателя в течение 700 миллисекунд, и скорость увеличилась, то пробой симистора
            triac_breakdown_flag = true;
            triac_breakdown_timer = millis();
          }
        }
      }

      if (triac_breakdown_flag == false && eng_obstruction_detected_flag == false) {
        regulator.setpoint = oboroti_po_regulytoru_dvigately;                //   заданная величина, которую должен поддерживать регулятор
        regulator.input = (unsigned long)(oboroti_po_pokazaniym_taho * 60);  //   сигнал с датчика (например температура, которую мы регулируем)
        regulator.getResultTimer();
        engpower = regulator.output;  //   выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
        dim.write(engpower);
        holl = 0;        //   обнуляем срабатывание датчика
        hool_count = 0;  //   обнуляем срабатывание счетчика для меню
      }
    }
  }
#endif

              //  ============================================================  Пробой симистора  ============================================================


#if (Flash_prog == 0 && menu_debug == 0)
  void triac_breakdown() {
    if (triac_breakdown_flag == true) {  //   дописать в управление через порт
      rew = false;
      if (millis() > triac_breakdown_timer + 300) {
        engpower = 0;
        Timer2.disableISR();
        Watchdog.reset();
        digitalWrite(rele_1, rew);                       //   переключаем реле  1
        digitalWrite(rele_2, rew);                       //   переключаем реле  2
        Watchdog.enable(RESET_MODE, WDT_PRESCALER_128);  //   перенастройка на сброс через 1 секунду
      }

      if (millis() > triac_breakdown_timer + 500) {
        triac_breakdown_timer = millis();
        triac_breakdown_timer_flag = !triac_breakdown_timer_flag;
      }
    }
  }
#endif