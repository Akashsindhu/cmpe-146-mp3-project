#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board_io.h"
#include "common_macros.h"
#include "delay.h"
#include "ff.h"
#include "gpio.h"
#include "task.h"

#include "adc_burst.h"
#include "gpio_isr.h"
#include "i2c.h"
#include "my_ssp0.h"
#include "queue.h"
#include "semphr.h"
#include "sj2_cli.h"
#include "uart_lab.h"

#include "lpc40xx.h"
#include "lpc_peripherals.h"

// scons --project="./MP3_project/"
// Python nxp-programmer/flash.py -d COM3 -i _build_MP3_project\MP3_project.bin
// Python nxp-programmer/flash.py -d COM4 -i _build_MP3_project\MP3_project.bin
#define DATA_SIZE 512
#define SONG_NAME_SIZE 16
#define MAX_SONG 20

QueueHandle_t Q_songname;
QueueHandle_t Q_songdata;
QueueHandle_t Q_displayname;

SemaphoreHandle_t cs_mutex;

struct song_s {
  char song_name[SONG_NAME_SIZE + 5];
  struct song_s *next_song;
  struct song_s *prev_song;
};

void mp3_reader_and_display_task(void *p);
// void mp3_vol_task(void *p);
void mp3_player_task(void *p);

void mp3_decoder__init(void);
void mp3_rst(void);
void decoder_cs(void);
void decoder_ds(void);
void decoder_dcs(void);
void decoder_dds(void);
void mp3_decoder__command_write(uint8_t addr, uint16_t command);
uint16_t mp3_decoder__command_read(uint8_t addr);

void init_lcd(void);
void lcd_ui(bool pause, uint8_t vol, char song[SONG_NAME_SIZE + 5]);
void start_screen_lcd(void);
void error_Screen_lcd(int error);

void config_uart_pins();
void set_ssp_pin_functions(void);
static void next_button_pressed(void);
static void prev_button_pressed(void);
static void pause_play_button_pressed(void);

static void vol_up(void);
static void vol_down(void);
void vol_change(void);

static gpio_s next_button, prev_button, pause_play_button, mode_switch, decoder_data_req;
static volatile bool next_pressed, prev_pressed, pause_pressed, up_pressed, down_pressed;
bool update_lcd = false;
volatile uint8_t volume = 0;

int main(void) {
  uint32_t peripheral_clock = clock__get_peripheral_clock_hz();

  uart_lab__init(UART__2, peripheral_clock, 9600);
  config_uart_pins();

  decoder_data_req = gpio__construct_with_function(GPIO__PORT_2, 2, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_input(decoder_data_req);

  // sj2_cli__init();
  set_ssp_pin_functions();
  my_ssp0__init(1);

  next_button = gpio__construct_as_input(GPIO__PORT_0, 22);
  prev_button = gpio__construct_as_input(GPIO__PORT_0, 0);
  pause_play_button = gpio__construct_as_input(GPIO__PORT_0, 1);
  mode_switch = gpio__construct_as_input(GPIO__PORT_0, 8);

  lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__GPIO, gpio0__interrupt_dispatcher);

  gpio0__attach_interrupt(22, GPIO_INTR__FALLING_EDGE, next_button_pressed);
  gpio0__attach_interrupt(0, GPIO_INTR__FALLING_EDGE, prev_button_pressed);
  gpio0__attach_interrupt(1, GPIO_INTR__FALLING_EDGE, pause_play_button_pressed);

  gpio__construct_as_input(GPIO__PORT_0, 7);
  gpio__construct_as_input(GPIO__PORT_0, 9);
  gpio0__attach_interrupt(7, GPIO_INTR__FALLING_EDGE, vol_up);
  gpio0__attach_interrupt(9, GPIO_INTR__FALLING_EDGE, vol_down);

  NVIC_EnableIRQ(GPIO_IRQn);

  Q_songname = xQueueCreate(1, (SONG_NAME_SIZE * sizeof(char)));
  Q_displayname = xQueueCreate(1, (SONG_NAME_SIZE * sizeof(char)));
  Q_songdata = xQueueCreate(1, (DATA_SIZE * sizeof(char)));

  cs_mutex = xSemaphoreCreateMutex();

  xTaskCreate(mp3_reader_and_display_task, "read_and_display", (512U * 8) / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  // xTaskCreate(mp3_vol_task, "volume_control", (512U * 4) / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(mp3_player_task, "mp3_player", (512U * 4) / sizeof(void *), NULL, PRIORITY_HIGH, NULL);
  vTaskStartScheduler();

  return 0;
}

void mp3_reader_and_display_task(void *p) {

  FILINFO fino;
  DIR dir;
  FIL file;
  UINT bytes_read = 0;
  FRESULT result;

  char songpath[SONG_NAME_SIZE + 14];
  char data[DATA_SIZE];
  char *check;

  uint8_t song_total = 0;
  uint8_t old_vol = 0;

  bool pause = true;
  bool start = true;
  bool first = true;
  bool update_song = false;

  next_pressed = false;
  prev_pressed = false;
  pause_pressed = false;
  up_pressed = false;
  down_pressed = false;

  struct song_s *first_song;
  struct song_s *current_song;

  if (FR_OK == f_opendir(&dir, "playlist")) {
    while (FR_OK == f_readdir(&dir, &fino) && fino.fname[0]) {
      check = strstr(fino.fname, ".mp3");
      if (check != NULL) {
        struct song_s *new_song = (struct song_s *)(malloc(sizeof(struct song_s)));
        strcpy(new_song->song_name, fino.fname);
        if (first) {
          first_song = new_song;
          current_song = new_song;
          first = false;
        } else {
          new_song->prev_song = current_song;
          current_song->next_song = new_song;
          current_song = new_song;
        }
        song_total++;
      }
    }
    f_closedir(&dir);
  }
  if (song_total == 0) {
    error_Screen_lcd(0);
    while (1) {
    }
  } else {
    first_song->prev_song = current_song;
    current_song->next_song = first_song;
    current_song = first_song;
  }

  init_lcd();

  while (1) {
    if (update_song) {
      f_close(&file);

      strcpy(songpath, "playlist/");
      strcat(songpath, current_song->song_name);
      printf("path %s \n", songpath);

      result = f_open(&file, songpath, (FA_READ));

      if (FR_OK != result) {
        error_Screen_lcd(1);
      }

      bytes_read = 0;
      update_song = false;

      mp3_rst();

    } else if (start) {
      vTaskDelay(1000);
      start_screen_lcd();
    }

    if (FR_OK == result) {
      while (!f_eof(&file) && !next_pressed && !prev_pressed && !pause_pressed && !pause && !up_pressed &&
             !down_pressed) {
        result = f_read(&file, &data, sizeof(data), &bytes_read);
        if (FR_OK != result) {
          error_Screen_lcd(1);
        }
        xQueueSend(Q_songdata, &data[0], portMAX_DELAY);
      }
      if (f_eof(&file) && !next_pressed && !prev_pressed && !pause_pressed && !pause) {
        pause = true;
        update_lcd = true;
      }
      vol_change();
    }

    do {

      if (next_pressed && !gpio__get(mode_switch)) {
        current_song = current_song->next_song;
        // printf("next %i \n", songnavigator);
        next_pressed = false;
        update_song = true;
        update_lcd = true;
      }

      if (prev_pressed && !gpio__get(mode_switch)) {
        current_song = current_song->prev_song;
        // printf("prev %i \n", songnavigator);
        prev_pressed = false;
        update_song = true;
        update_lcd = true;
      }

      if (pause_pressed && !gpio__get(mode_switch)) {
        // printf("pause toggled \n");
        if (pause) {
          pause = false;
        } else {
          pause = true;
        }
        pause_pressed = false;
        update_lcd = true;
      }

      if (up_pressed && !gpio__get(mode_switch)) {
        if (volume < 100) {
          volume += 5;
        }
        up_pressed = false;
        update_lcd = true;
      }

      if (down_pressed && !gpio__get(mode_switch)) {
        if (volume > 0) {
          volume -= 5;
        }
        down_pressed = false;
        update_lcd = true;
      }

      vTaskDelay(200);

      if (update_lcd) {
        uart_lab__polled_put(UART__2, 0x7C);
        uart_lab__polled_put(UART__2, 0x2D); // clear display
        vTaskDelay(1);
        lcd_ui(pause, volume, current_song->song_name);
        update_lcd = false;
        start = false;
      }

    } while (pause);
  }
}
/*
void mp3_vol_task(void *p) {
  adc__initialize();

  adc__enable_burst_mode(ADC__CHANNEL_2);

  gpio_s adcin;
  adcin = gpio__construct_with_function(GPIO__PORT_0, 25, GPIO__FUNCTION_1);
  gpio__set_as_input(adcin);
  LPC_IOCON->P0_25 &= ~(3 << 3); // clear pull up and pull down resistors
  uint16_t knob_value = 0;
  while (1) {
    knob_value = adc__get_adc_value_with_burst_mode(ADC__CHANNEL_2);
    printf("%i \n", knob_value);
    vTaskDelay(500);
  }

}
*/
void mp3_player_task(void *p) {
  mp3_decoder__init();

  char data[DATA_SIZE];

  while (1) {
    xQueueReceive(Q_songdata, &data[0], portMAX_DELAY);

    decoder_dcs();
    for (int i = 0; i < DATA_SIZE; i++) {
      my_ssp0__exchange_byte(data[i]);
      while (!gpio__get(decoder_data_req)) {
        vTaskDelay(1);
      }
    }
    decoder_dds();
  }
}

//===============================================MP3 Decoder Functions

void mp3_decoder__init(void) {

  uint8_t to_decoder = 0;
  uint16_t final_to_decoder = 0;

  gpio_s rst;
  rst = gpio__construct(GPIO__PORT_2, 5);

  to_decoder = 255 / 100 * volume;
  to_decoder = 255 - to_decoder;

  final_to_decoder = (to_decoder << 0) & 0x00FF;
  final_to_decoder |= (to_decoder << 8);

  gpio__set(rst);
  vTaskDelay(10);
  gpio__reset(rst);
  vTaskDelay(10);
  gpio__set(rst);

  decoder_cs();
  mp3_decoder__command_write(0x0, 0x4880);           // mode
  mp3_decoder__command_write(0x3, 0x6000);           // clock
  mp3_decoder__command_write(0xB, final_to_decoder); // volume
  decoder_ds();
}

void vol_change(void) {
  uint8_t to_decoder = 0;
  uint16_t final_to_decoder = 0;

  to_decoder = 255 / 100 * (volume + 55);
  to_decoder = 255 - to_decoder;

  final_to_decoder = (to_decoder << 0) & 0x00FF;
  final_to_decoder |= (to_decoder << 8);

  decoder_cs();
  mp3_decoder__command_write(0xB, final_to_decoder); // volume
  decoder_ds();
}

void mp3_rst(void) {

  uint8_t to_decoder = 0;
  uint16_t final_to_decoder = 0;

  to_decoder = 255 / 100 * volume;
  to_decoder = 255 - to_decoder;

  final_to_decoder = (to_decoder << 0) & 0x00FF;
  final_to_decoder |= (to_decoder << 8);

  decoder_cs();
  mp3_decoder__command_write(0x0, 0x4880);           // mode
  mp3_decoder__command_write(0x3, 0x6000);           // clock
  mp3_decoder__command_write(0xB, final_to_decoder); // volume
  decoder_ds();
}

void mp3_cancel(void) {
  mp3_decoder__command_write(0x0, 0x4880); // mode w/ cancel bit
}

void decoder_cs(void) {
  gpio_s cs;
  cs = gpio__construct(GPIO__PORT_0, 6);
  gpio__reset(cs);
}

void decoder_ds(void) {
  gpio_s cs;
  cs = gpio__construct(GPIO__PORT_0, 6);
  gpio__set(cs);
}

void decoder_dcs(void) {
  gpio_s data_cs;
  data_cs = gpio__construct(GPIO__PORT_2, 0);
  gpio__reset(data_cs);
}

void decoder_dds(void) {
  gpio_s data_cs;
  data_cs = gpio__construct(GPIO__PORT_2, 0);
  gpio__set(data_cs);
}

void mp3_decoder__command_write(uint8_t addr, uint16_t command) {
  uint8_t write = 0x02;

  while (!gpio__get(decoder_data_req)) {
  }

  decoder_cs();

  my_ssp0__exchange_byte(write);
  my_ssp0__exchange_byte(addr);
  my_ssp0__exchange_byte(command >> 8);
  my_ssp0__exchange_byte(command >> 0);

  decoder_ds();

  while (!gpio__get(decoder_data_req)) {
  }
}

uint16_t mp3_decoder__command_read(uint8_t addr) {
  uint8_t read = 0x03;
  uint16_t data = 0;

  while (!gpio__get(decoder_data_req)) {
  }

  decoder_cs();

  my_ssp0__exchange_byte(read);
  my_ssp0__exchange_byte(addr);
  data = my_ssp0__exchange_byte(0xFF);
  data = (data << 8) | my_ssp0__exchange_byte(0xFF);

  decoder_ds();

  while (!gpio__get(decoder_data_req)) {
  }

  return data;
}

//==========================================UART, LCD, and Buttons Functions

void init_lcd(void) {
  uart_lab__polled_put(UART__2, 0x7C);
  uart_lab__polled_put(UART__2, 0x08); // soft reset
  vTaskDelay(1);
  uart_lab__polled_put(UART__2, 0x7C);
  uart_lab__polled_put(UART__2, 0x18);
  uart_lab__polled_put(UART__2, 100); // contrast
  vTaskDelay(1);
  uart_lab__polled_put(UART__2, 0x7C);
  uart_lab__polled_put(UART__2, 0x0D); // baud
  vTaskDelay(1);
  uart_lab__polled_put(UART__2, 0x7C);
  uart_lab__polled_put(UART__2, 0x2B);
  uart_lab__polled_put(UART__2, 0x9D); // red
  uart_lab__polled_put(UART__2, 0xBB); // green
  uart_lab__polled_put(UART__2, 0xD9); // blue
  vTaskDelay(1);
}

void lcd_ui(bool pause, uint8_t vol, char song[SONG_NAME_SIZE + 5]) {
  char paused[] = "PAUSED";
  char play[] = "PLAYING";
  char volu[] = "VOL";
  char volval[3];
  bool endofname = false;

  for (int i = 0; i < 16; i++) {

    if (song[i] == '.') {
      endofname = true;
      uart_lab__polled_put(UART__2, ' ');
    } else if (!endofname) {
      uart_lab__polled_put(UART__2, song[i]);
    } else {
      uart_lab__polled_put(UART__2, ' ');
    }
  }

  if (pause) {
    for (int i = 0; i < 8; i++) {
      if (i < sizeof(paused) - 1) {
        uart_lab__polled_put(UART__2, paused[i]);
      } else {
        uart_lab__polled_put(UART__2, ' ');
      }
    }
  } else {
    for (int i = 0; i < 8; i++) {
      if (i < sizeof(play) - 1) {
        uart_lab__polled_put(UART__2, play[i]);
      } else {
        uart_lab__polled_put(UART__2, ' ');
      }
    }
  }
  for (int i = 0; i < sizeof(volu) - 1; i++) {
    uart_lab__polled_put(UART__2, volu[i]);
  }

  uart_lab__polled_put(UART__2, ' ');

  if (vol >= 100) {
    itoa(vol, volval, 10);
    uart_lab__polled_put(UART__2, volval[0]);
    uart_lab__polled_put(UART__2, volval[1]);
    uart_lab__polled_put(UART__2, volval[2]);
  } else if (vol >= 10) {
    itoa(vol, volval, 10);
    uart_lab__polled_put(UART__2, ' ');
    uart_lab__polled_put(UART__2, volval[0]);
    uart_lab__polled_put(UART__2, volval[1]);
  } else {
    itoa(vol, volval, 10);
    uart_lab__polled_put(UART__2, ' ');
    uart_lab__polled_put(UART__2, ' ');
    uart_lab__polled_put(UART__2, volval[0]);
  }
}

void start_screen_lcd(void) {
  char name[] = " THE WOODEN BOX ";
  char mp3[] = "   MP3 PLAYER   ";

  for (int i = 0; i < 16; i++) {
    uart_lab__polled_put(UART__2, name[i]);
  }
  for (int i = 0; i < 16; i++) {
    uart_lab__polled_put(UART__2, mp3[i]);
  }
}
void error_Screen_lcd(int error) {
  char errormessage[] = " ERROR  OCCURED ";
  char playlisterror[] = " SONGS NOT FOUND ";
  char songerror[] = " SONG UNREADABLE ";

  while (1) {
    uart_lab__polled_put(UART__2, 0x7C);
    uart_lab__polled_put(UART__2, 0x2D); // clear display

    for (int i = 0; i < 16; i++) {
      uart_lab__polled_put(UART__2, errormessage[i]);
    }
    if (error == 0) {
      for (int i = 0; i < 16; i++) {
        uart_lab__polled_put(UART__2, playlisterror[i]);
      }
    } else if (error == 1) {
      for (int i = 0; i < 16; i++) {
        uart_lab__polled_put(UART__2, songerror[i]);
      }
    }

    vTaskDelay(1000);
  }
}

void config_uart_pins() {
  gpio__construct_with_function(GPIO__PORT_2, 8, GPIO__FUNCTION_2); // Tx
  gpio__construct_with_function(GPIO__PORT_2, 9, GPIO__FUNCTION_2); // Rx
  LPC_IOCON->P2_9 &= ~(3 << 3);
  LPC_IOCON->P2_9 |= (2 << 3); // pull up res
}

void set_ssp_pin_functions(void) {
  gpio_s cs, data_cs;

  cs = gpio__construct_with_function(GPIO__PORT_0, 6, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_output(cs);
  gpio__set(cs);

  data_cs = gpio__construct_with_function(GPIO__PORT_2, 0, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_output(data_cs);
  gpio__set(data_cs);

  gpio__construct_with_function(GPIO__PORT_0, 17, GPIO__FUNCTION_2); // miso

  gpio__construct_with_function(GPIO__PORT_0, 18, GPIO__FUNCTION_2); // mosi

  gpio__construct_with_function(GPIO__PORT_0, 15, GPIO__FUNCTION_2); // sck
}

void next_button_pressed(void) {
  if (!update_lcd && !next_pressed) {
    next_pressed = true;
  }
}

void prev_button_pressed(void) {
  if (!update_lcd && !prev_pressed) {
    prev_pressed = true;
  }
}

void pause_play_button_pressed(void) {
  if (!update_lcd && !pause_pressed) {
    pause_pressed = true;
  }
}

void vol_up(void) {
  if (!update_lcd && !up_pressed) {
    up_pressed = true;
  }
}
void vol_down(void) {
  if (!update_lcd && !down_pressed) {
    down_pressed = true;
  }
}
