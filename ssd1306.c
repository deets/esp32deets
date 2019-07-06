// Copyright: 2018, Diez B.Roggisch, Berlin, all rights reserved

#include "ssd1306.h"
#include "ssd1306_consts.h"
#include "common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <math.h>
#include <sys/param.h>


//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
static void ssd1306_cmd(ssd1306_display_t *display, const uint8_t* cmd, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8 * len;                     //Command is 8 bits
    t.tx_buffer=cmd;               //The data is the cmd itself
    t.user=(void*)display;                //D/C needs to be set to 1
    display->dc_level = 0; //D/C needs to be set to 0
    ret=spi_device_transmit(display->spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void ssd1306_data(ssd1306_display_t *display, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)display;                //D/C needs to be set to 1
    display->dc_level = 1;
    ret=spi_device_transmit(display->spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
static void ssd1306_spi_pre_transfer_callback(spi_transaction_t *t)
{
  ssd1306_display_t *display = (ssd1306_display_t*)t->user;
  gpio_set_level(display->dc_pin, display->dc_level);
}

//Initialize the display
static void ssd1306_setup(ssd1306_display_t *display)
{
    //Reset the display
    gpio_set_level(display->rst_pin, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(display->rst_pin, 0);
    vTaskDelay(1);
    gpio_set_level(display->rst_pin, 1);
    vTaskDelay(1);

    uint8_t cmd1[] = {SSD1306_DISPLAYOFF, SSD1306_SETDISPLAYCLOCKDIV, 0x80};
    uint8_t cmd2[] = {SSD1306_SETMULTIPLEX, SSD1306_LCDHEIGHT - 1};
    uint8_t cmd3[] = {SSD1306_SETDISPLAYOFFSET, 0x00, SSD1306_SETSTARTLINE | 0x00};
    uint8_t cmd4[] = {SSD1306_CHARGEPUMP, 0x14};
    uint8_t cmd5[] = {SSD1306_MEMORYMODE, 0x00};
    uint8_t cmd6[] = {SSD1306_SEGREMAP | 0x1, SSD1306_COMSCANDEC};
    uint8_t cmd7[] = {SSD1306_SETCOMPINS, 0x12};
    uint8_t cmd8[] = {SSD1306_SETCONTRAST, 0xCF}; // SSD1306_SWITCHCAPV
    uint8_t cmd9[] = {SSD1306_SETPRECHARGE, 0xF1}; // SSD1306_SWITCHCAPV
    uint8_t cmdA[] = {SSD1306_SETVCOMDETECT, 0x40};
    uint8_t cmdB[] = {SSD1306_DISPLAYALLON_RESUME, SSD1306_NORMALDISPLAY, SSD1306_DEACTIVATE_SCROLL,
                      SSD1306_DISPLAYON};
    uint8_t cmdC[] = {SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH - 1};
    uint8_t cmdD[] = {SSD1306_PAGEADDR, 0, 7};

    ssd1306_cmd(display, cmd1, sizeof(cmd1));
    ssd1306_cmd(display, cmd2, sizeof(cmd2));
    ssd1306_cmd(display, cmd3, sizeof(cmd3));
    ssd1306_cmd(display, cmd4, sizeof(cmd4));
    ssd1306_cmd(display, cmd5, sizeof(cmd5));
    ssd1306_cmd(display, cmd6, sizeof(cmd6));
    ssd1306_cmd(display, cmd7, sizeof(cmd7));
    ssd1306_cmd(display, cmd8, sizeof(cmd8));
    ssd1306_cmd(display, cmd9, sizeof(cmd9));
    ssd1306_cmd(display, cmdA, sizeof(cmdA));
    ssd1306_cmd(display, cmdB, sizeof(cmdB));
    ssd1306_cmd(display, cmdC, sizeof(cmdC));
    ssd1306_cmd(display, cmdD, sizeof(cmdD));
}


void ssd1306_draw_pixel(ssd1306_display_t *display, int x, int y)
{
  if(x < 0 || x >= SSD1306_LCDWIDTH || y < 0 || y >=SSD1306_LCDHEIGHT)
  {
    return;
  }
  display->frame[(x >> 5) + y * (SSD1306_LCDWIDTH / 32)] |= 1 << (x & 0x1f);
}


int ssd1306_init_static(
  ssd1306_display_t *display,
  int cs, int clk, int mosi,
  int dc, int rst,
  int spi_speed
  )
{
    esp_err_t ret;
    display->dc_pin = dc;
    display->rst_pin = rst;
    display->width = SSD1306_LCDWIDTH;
    display->height = SSD1306_LCDHEIGHT;
    display->frame_byte_size = SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8;


    //Initialize non-SPI GPIOs
    gpio_set_direction(dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(rst, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=mosi,
        .sclk_io_num=clk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=display->frame_byte_size,
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=spi_speed,           //Clock out at 2 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=cs,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=ssd1306_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &display->spi);
    ESP_ERROR_CHECK(ret);

    //Initialize the LCD
    ssd1306_setup(display);

    display->frame = malloc(display->frame_byte_size);
    display->raw_frame = malloc(display->frame_byte_size);

    return 0;
}


void ssd1306_pixels_to_raw(uint32_t* frame, uint8_t* raw_frame)
{
  for(size_t y=0; y < SSD1306_LCDHEIGHT; ++y)
  {
    for(size_t xword = 0; xword < (SSD1306_LCDWIDTH / 32); ++xword)
    {
      uint32_t word = *frame++;
      for(size_t xbit=0; xbit < 32; ++xbit)
      {
        if(word & 1)
        {
          raw_frame[(xword << 5) + xbit + (y/8)*SSD1306_LCDWIDTH] |= (1 << (y&7));
        }
        word >>= 1;
      }
    }
  }
}


void ssd1306_update(ssd1306_display_t *display)
{
    uint8_t cmd1[] = {SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH - 1};
    uint8_t cmd2[] = {SSD1306_PAGEADDR, 0, 7};
    ssd1306_cmd(display, cmd1, sizeof(cmd1));
    ssd1306_cmd(display, cmd2, sizeof(cmd2));
    memset(display->raw_frame, 0, display->frame_byte_size);
    ssd1306_pixels_to_raw(display->frame, display->raw_frame);
    ssd1306_data(display, display->raw_frame, display->frame_byte_size);
    gpio_set_level(display->dc_pin, 0);
}


void ssd1306_clear(ssd1306_display_t *display)
{
  memset(display->frame, 0, display->frame_byte_size);
}


void ssd1306_draw_vertical_line(ssd1306_display_t* display, int x, int y, int y2)
{
  // normalise y-coordinates
  if(y > y2)
  {
    int h = y;
    y = y2;
    y2 = h;
  }
  // don't show invisible lines
  if(x < 0 || x > display->width - 1 || y == y2 || y > display->height - 1 || y2 < 0)
  {
    return;
  }
  y = max(y, 0); // clip to top
  y2 = min(y2, display->height - 1); // clip to bottom
  for(int yrun = y; yrun <= y2; ++yrun)
  {
    ssd1306_draw_pixel(display, x, yrun);
  }
}


void ssd1306_draw_horizontal_line(ssd1306_display_t* display, int x, int x2, int y)
{
  if(y < 0 || y >= SSD1306_LCDHEIGHT)
  {
    return;
  }

  if(x > x2)
  {
    int h = x;
    x = x2;
    x2 = h;
  }
  x = MAX(0, x);
  x2 = MIN(SSD1306_LCDWIDTH - 1, x2);
  for(int i=x; i <= x2; ++i)
  {
    ssd1306_draw_pixel(display, i, y);
  }
}
