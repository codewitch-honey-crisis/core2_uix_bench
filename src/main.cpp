#include <Arduino.h>

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include <esp_lcd_panel_ili9342.h>
#include <m5core2_power.hpp>
#define LCD_SPI_HOST    SPI3_HOST
#define LCD_DMA
#define LCD_BCKL_ON_LEVEL 1
#define LCD_BCKL_OFF_LEVEL !LCD_BCKL_ON_LEVEL
#define LCD_PIN_NUM_MOSI 23
#define LCD_PIN_NUM_CLK 18
#define LCD_PIN_NUM_CS 5
#define LCD_PIN_NUM_DC 15
#define LCD_PANEL esp_lcd_new_panel_ili9342
#define LCD_HRES 320
#define LCD_VRES 240
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_BGR
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_GAP_X 0
#define LCD_GAP_Y 0
#define LCD_MIRROR_X false
#define LCD_MIRROR_Y false
#define LCD_INVERT_COLOR true
#define LCD_SWAP_XY false
#include <ft6336.hpp>
#include <gfx.hpp>
#include <uix.hpp>
using namespace arduino;
using namespace gfx;
using namespace uix;
// downloaded from fontsquirrel.com and header generated with
// https://honeythecodewitch.com/gfx/generator
#define ARCHITECTS_DAUGHTER_IMPLEMENTATION
#include <assets/architects_daughter.hpp>
static const open_font& text_font = architects_daughter;
// downloaded for free from pngtree.com and header generated with
// https://honeythecodewitch.com/gfx/generator
#define SKETCHY_IMPLEMENTATION
#include <assets/sketchy.hpp>

// for Core2 power management
static m5core2_power power;
static ft6336<320,280> touch(Wire1);

// declare the format of the screen
using screen_t = screen<rgb_pixel<16>>;
using color_t = color<typename screen_t::pixel_type>;
// for access to RGB8888 colors which controls use
using color32_t = color<rgba_pixel<32>>;

extern screen_t anim_screen;

using canvas_t = canvas<typename screen_t::control_surface_type>;
using label_t = label<typename screen_t::control_surface_type>;
// UIX allows you to use two buffers for maximum DMA efficiency
// you don't have to, but performance is significantly better
// declare 64KB across two buffers for transfer
constexpr static const int lcd_buffer_size = 32 * 1024;
static uint8_t lcd_buffer1[lcd_buffer_size];
static uint8_t lcd_buffer2[lcd_buffer_size];
// this is the handle from the esp panel api
static esp_lcd_panel_handle_t lcd_handle;

// the main screen
screen_t anim_screen({LCD_HRES, LCD_VRES}, sizeof(lcd_buffer1), lcd_buffer1, lcd_buffer2);

// the controls
static canvas_t anim_canvas(anim_screen);
static label_t fps_label(anim_screen);
// tell UIX the DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    anim_screen.flush_complete();
    return true;
}
// tell the lcd panel api to transfer data via DMA
static void uix_on_flush(const rect16& bounds, const void* bmp, void* state) {
    int x1 = bounds.x1, y1 = bounds.y1, x2 = bounds.x2 + 1, y2 = bounds.y2 + 1;
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2, y2, (void*)bmp);
}

// initialize the screen using the esp panel API
static void lcd_panel_init() {
#ifdef LCD_PIN_NUM_BCKL
    pinMode(LCD_PIN_NUM_BCKL, OUTPUT);
#endif
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = LCD_PIN_NUM_CLK;
    buscfg.mosi_io_num = LCD_PIN_NUM_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(lcd_buffer1) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = LCD_PIN_NUM_DC,
    io_config.cs_gpio_num = LCD_PIN_NUM_CS,
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
#ifdef LCD_PIN_NUM_RST
    panel_config.reset_gpio_num = LCD_PIN_NUM_RST;
#else
    panel_config.reset_gpio_num = -1;
#endif
    panel_config.color_space = LCD_COLOR_SPACE;
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    esp_lcd_new_panel_ili9342(io_handle, &panel_config, &lcd_handle);

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
#ifdef LCD_PIN_NUM_BCKL
    digitalWrite(LCD_PIN_NUM_BCKL, LCD_BCKL_OFF_LEVEL);
#endif
    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, LCD_SWAP_XY);
    esp_lcd_panel_set_gap(lcd_handle, LCD_GAP_X, LCD_GAP_Y);
    esp_lcd_panel_mirror(lcd_handle, LCD_MIRROR_X, LCD_MIRROR_Y);
    esp_lcd_panel_invert_color(lcd_handle, LCD_INVERT_COLOR);
    // Turn on the screen
    esp_lcd_panel_disp_off(lcd_handle, false);
    // Turn on backlight (Different LCD screens may need different levels)
#ifdef LCD_PIN_NUM_BCKL
    digitalWrite(LCD_PIN_NUM_BCKL, LCD_BCKL_ON_LEVEL);
#endif
}
// initialize the screens and controls
static void screen_init() {
    const rgba_pixel<32> transparent(0,0,0,0);

    anim_canvas.bounds(srect16(0,0,319,239).center(anim_screen.bounds()));
    anim_canvas.on_paint_callback([](canvas_t::control_surface_type& destination,const srect16& clip, void* state){
        // destinaion = draw surface
        // clip = clipping rectangle
        
        // these are all kept beteen calls:
        constexpr static const size_t count = 10;
        constexpr static const int16_t radius = 25;
        static bool init=false;
        static spoint16 pts[count]; // locations
        static spoint16 dts[count]; // deltas
        static rgba_pixel<32> cls[count]; // colors
        // first time, we must initialize
        if(!init) {
            init = true;
            for(size_t i = 0;i<count;++i) {
                // start at the center
                pts[i]=spoint16(destination.dimensions().width/2,destination.dimensions().height/2);
                dts[i]={0,0};
                // random deltas. Retry on (dx=0||dy=0)
                while(dts[i].x==0||dts[i].y==0) {
                    dts[i].x=(rand()%5)-2;
                    dts[i].y=(rand()%5)-2;        
                }
                // random color RGBA8888
                cls[i]=rgba_pixel<32>((rand()%255),(rand()%255),(rand()%255),(rand()%224)+32);
            }
        }
        // draw a checkerboard
        for(int y = 0;y<destination.dimensions().height;y+=16) {
            for(int x = 0;x<destination.dimensions().width;x+=16) {
                srect16 r(x,y,x+16,y+16);
                if(r.intersects(clip)) {
                    bool w = 0 != ((x + y) % 32);
                    if(w) {
                        draw::filled_rectangle(destination,r,color_t::white);
                    }
                }
            }    
        }
        for(size_t i = 0;i<count;++i) {
            spoint16& pt = pts[i];
            spoint16& d = dts[i];
            srect16 r(pt,radius);
            if(clip.intersects(r)) {
                rgba_pixel<32>& col = cls[i];
                draw::filled_ellipse(destination,r,col,&clip);
            }
            // move the circle
            pt.x+=d.x;
            pt.y+=d.y;
            // if it is about to hit the edge, invert 
            // the respective deltas
            if(pt.x+d.x+-radius<=0 || pt.x+d.x+radius>=destination.bounds().x2) {
                d.x=-d.x;
            } 
            if(pt.y+d.y+-radius<=0 || pt.y+d.y+radius>=destination.bounds().y2) {
                d.y=-d.y;
            }
        }
    });
    fps_label.text_color(color32_t::red);
    fps_label.text_open_font(&text_font);
    fps_label.text_line_height(40);
    fps_label.padding({0,0});
    rgba_pixel<32> bg(0,0,0,224);
    fps_label.background_color(bg);
    fps_label.text_justify(uix_justify::bottom_right);
    fps_label.bounds(srect16(anim_screen.dimensions().width/2+1,anim_screen.bounds().y2-fps_label.text_line_height()+2,anim_screen.bounds().x2,anim_screen.bounds().y2));
    anim_screen.register_control(anim_canvas);
    anim_screen.register_control(fps_label);
    anim_screen.background_color(color_t::black);
    anim_screen.on_flush_callback(uix_on_flush);
}
// set up the hardware
void setup() {
    Serial.begin(115200);
    power.initialize();
    touch.initialize();
    touch.rotation(0);
    // init the display
    lcd_panel_init();
    // init the UI screen
    screen_init();
}
// keep our stuff up to date and responsive
void loop() {
    static int frames = 0;
    static char szfps[16];
    // if we're on the animation screen

    static uint32_t fps_ts = 0;
    uint32_t ms = millis();
        // force the canvas to repaint
        anim_canvas.invalidate();
        ++frames;
    
    if(ms>fps_ts+1000) {
        fps_ts = ms;
        snprintf(szfps,sizeof(szfps),"fps: %d",frames);
        fps_label.text(szfps);
        frames = 0;
    }
    
    anim_screen.update();
}