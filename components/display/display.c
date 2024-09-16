/**
 * @file   display.c
 * @brief  This file contains function definitions for the display.
 *
 * @copyright Copyright PND(c) 2022
 *
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "dispcolor.h"
#include "display.h"
#include "gc9a01.h"
#include "font.h"

#include "logos.h"
#include "numbers.h"
#include "sim7000a.h"

#define DISP_REFRESH_TASK_PERIOD            (33)          /*!< DISP application task scheduling rate in ms */
#define DISP_CARRIER_LOGO_X_OFFSET          (85)
#define DISP_CARRIER_LOGO_Y_OFFSET          (1)
#define DISP_CARRIER_BARS_X_OFFSET          (120)
#define DISP_CARRIER_BARS_Y_OFFSET          (32)
#define DISP_GPS_GLYPH_X_OFFSET             (80)
#define DISP_GPS_GLYPH_Y_OFFSET             (200)
#define DISP_GPS_BARS_X_OFFSET              (120)
#define DISP_GPS_BARS_Y_OFFSET              (235)
#define DISP_SPEED_GLYPH_TENS_X_OFFSET      (25)
#define DISP_SPEED_GLYPH_TENS_Y_OFFSET      (50)
#define DISP_SPEED_GLYPH_ONES_X_OFFSET      (115)
#define DISP_SPEED_GLYPH_ONES_Y_OFFSET      (50)

static const char *DBG_TAG = "disp";

// Variables pertaining to screen drawing
static unsigned int disp_tick = 0;

static SIM_Carrier_e last_carrier = SIM_CARRIER_SEARCHING;
static SIM_Carrier_e curr_carrier = SIM_CARRIER_UNKNOWN;

static unsigned int last_rssi = 100;
static unsigned int curr_rssi = 0;

static SIM_GnssFixStatus_e last_gps_fix = SIM_GNSS_FIX;
static SIM_GnssFixStatus_e curr_gps_fix = SIM_GNSS_NO_FIX;

static unsigned int last_siv = 100;
static unsigned int curr_siv = 0;
static unsigned int last_siu = 100;
static unsigned int curr_siu = 0;

static unsigned int last_sog_mph = 99;
static unsigned int curr_sog_mph = 0;

static unsigned int last_cog_deg = 0;
static unsigned int curr_cog_deg = 0;

void disp_application_task(void* args);
void disp_draw_carrier_glyph(void);
void disp_update_cell_signal_strength(void);
void disp_draw_gps_glyph(void);
void disp_update_gps_sats_in_view(void);

void DISP_StartDevice(void) {
  // Set default logging level
  esp_log_level_set("disp", ESP_LOG_INFO);

  dispcolor_Init(240, 240);
  dispcolor_ClearScreen();
  dispcolor_Update();
}

void DISP_StartAppTask(void) {
  xTaskCreate(disp_application_task, "display_application_task", 4096, NULL, 5, NULL);
}

void DISP_OnEvent(DisplayEvent_e event, unsigned int data) {
  switch (event) {
    case DISP_EV_ACTIVE_CARRIER: {
      switch (data) {
        case SIM_CARRIER_UNKNOWN: {
          curr_carrier = SIM_CARRIER_UNKNOWN;
        } break;

        case SIM_CARRIER_SEARCHING: {
          curr_carrier = SIM_CARRIER_SEARCHING;
        } break;

        case SIM_CARRIER_VZW: {
          curr_carrier = SIM_CARRIER_VZW;
        } break;

        case SIM_CARRIER_ATT: {
          curr_carrier = SIM_CARRIER_ATT;
        } break;

        case SIM_CARRIER_TMO: {
          curr_carrier = SIM_CARRIER_TMO;
        } break;

        default: {
          ESP_LOGW(DBG_TAG, "Invalid DISP_EV_ACTIVE_CARRIER received: %d", data);
        }
      }
    } break;

    case DISP_EV_CURR_CSQ: {
      curr_rssi = data;
    } break;

    case DISP_EV_SAT_FIX: {
      curr_gps_fix = data;
    } break;

    case DISP_EV_SAT_IN_VIEW: {
      curr_siv = data;
    } break;

    case DISP_EV_SAT_IN_USE: {
      curr_siu = data;
    } break;
    
    case DISP_EV_SPEED_MPH: {
      curr_sog_mph = data;
    } break;

    case DISP_EV_COG_DEGREES: {
      curr_cog_deg = data;
    } break;

    default: {
      // Nothing to do!
    };
  }
}

void disp_draw_carrier_glyph(void) {
    DISP_Logo_t const * carrier_glyph = NULL;
    uint16_t concat;
    unsigned int x = DISP_CARRIER_LOGO_X_OFFSET;
    unsigned int y = DISP_CARRIER_LOGO_Y_OFFSET;
    
    if (curr_carrier != last_carrier) {
      ESP_LOGI(DBG_TAG, "Current carrier updated: (%d) -> (%d)", last_carrier, curr_carrier);
      last_carrier = curr_carrier;
      
      // Get pointer to logo struct
      switch (curr_carrier) {
        case SIM_CARRIER_UNKNOWN: {
          carrier_glyph = &logo_unknown;
        } break;

        case SIM_CARRIER_SEARCHING: {
          carrier_glyph = &logo_vzw;
        } break;

        case SIM_CARRIER_VZW: {
          carrier_glyph = &logo_vzw;
        } break;

        case SIM_CARRIER_ATT: {
          carrier_glyph = &logo_att;
        } break;

        case SIM_CARRIER_TMO: {
          carrier_glyph = &logo_tmo;
        } break;

        default: {
          // Nothing to do
        }
      }

      if (carrier_glyph != NULL) {
        ESP_LOGI(DBG_TAG, "Drawing cellular glyph");
        for (unsigned int i = 0; i < ((carrier_glyph->width * carrier_glyph->height * carrier_glyph->bytes_per_pixel) - 1); i += carrier_glyph->bytes_per_pixel) {
          if (x >= carrier_glyph->width + DISP_CARRIER_LOGO_X_OFFSET) {
            x = DISP_CARRIER_LOGO_X_OFFSET;
            y++;
          }
          uint16_t concat = ((carrier_glyph->pixel_data[i+1] << 8) + carrier_glyph->pixel_data[i]);
          dispcolor_DrawPixel(x++, y, concat);  
        }
      }

    }
}

void disp_update_cell_signal_strength(void) {
  if (curr_rssi != last_rssi) {
    ESP_LOGI(DBG_TAG, "Received new RSSI: (%d) -> (%d)", last_rssi, curr_rssi);
    last_rssi = curr_rssi;

    dispcolor_FillRectangle(120, 12, 148, 32, BLACK);  // Blank it

    // Empty bars
    dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 4, DISP_CARRIER_BARS_Y_OFFSET, WHITE);
    dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 6, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 10, DISP_CARRIER_BARS_Y_OFFSET, WHITE);
    dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 12, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 16, DISP_CARRIER_BARS_Y_OFFSET, WHITE);
    dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 18, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 22, DISP_CARRIER_BARS_Y_OFFSET, WHITE);
    dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 24, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 28, DISP_CARRIER_BARS_Y_OFFSET, WHITE);

    int bars = 0;
    if (curr_rssi <= 15 || curr_rssi >= 32) {
      bars = 0;
    } else if (curr_rssi <= 15) {
      bars = 1;
    } else if (curr_rssi <= 20) {
      bars = 2;
    } else if (curr_rssi <= 25) {
      bars = 3;
    } else if (curr_rssi <= 30) {
      bars = 4;
    } else {
      bars = 5;
    }

    switch (bars) {
      case 5: {
        dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 24, DISP_CARRIER_BARS_Y_OFFSET - 20, DISP_CARRIER_BARS_X_OFFSET + 28, DISP_CARRIER_BARS_Y_OFFSET, WHITE);  // bar 5
      }  // fallthrough-intentional

      case 4: {
        dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 18, DISP_CARRIER_BARS_Y_OFFSET - 16, DISP_CARRIER_BARS_X_OFFSET + 22, DISP_CARRIER_BARS_Y_OFFSET, WHITE);  // bar 4
      }  // fallthrough-intentional

      case 3: {
        dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 12, DISP_CARRIER_BARS_Y_OFFSET - 12, DISP_CARRIER_BARS_X_OFFSET + 16, DISP_CARRIER_BARS_Y_OFFSET, WHITE);  // bar 3
      }  // fallthrough-intentional

      case 2: {
        dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 6, DISP_CARRIER_BARS_Y_OFFSET - 8, DISP_CARRIER_BARS_X_OFFSET + 10, DISP_CARRIER_BARS_Y_OFFSET, WHITE);  // bar 2
      }  // fallthrough-intentional
      
      case 1: {
        dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET, DISP_CARRIER_BARS_Y_OFFSET - 4, DISP_CARRIER_BARS_X_OFFSET + 4, DISP_CARRIER_BARS_Y_OFFSET, WHITE);  // bar 1
      }  // fallthrough-intentional

      case 0: {
        // Nothing to do
      } break;
    
      default: {
        ESP_LOGW(DBG_TAG, "Unknown bars: (%d)", bars);
      }
    }

    if (curr_rssi == 99) {
      dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 4, DISP_CARRIER_BARS_Y_OFFSET, RED);
      dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 6, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 10, DISP_CARRIER_BARS_Y_OFFSET, RED);
      dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 12, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 16, DISP_CARRIER_BARS_Y_OFFSET, RED);
      dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 18, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 22, DISP_CARRIER_BARS_Y_OFFSET, RED);
      dispcolor_FillRectangle(DISP_CARRIER_BARS_X_OFFSET + 24, DISP_CARRIER_BARS_Y_OFFSET - 2, DISP_CARRIER_BARS_X_OFFSET + 28, DISP_CARRIER_BARS_Y_OFFSET, RED);

    }
  }
}

void disp_draw_gps_glyph(void) {
  DISP_Logo_t const * gps_glyph = NULL;

  int x = DISP_GPS_GLYPH_X_OFFSET;
  int y = DISP_GPS_GLYPH_Y_OFFSET;
  uint16_t concat;

  if (last_gps_fix != curr_gps_fix) {
    ESP_LOGI(DBG_TAG, "GPS fix status updated (%d) -> (%d)", last_gps_fix, curr_gps_fix);
    last_gps_fix = curr_gps_fix;

    switch (curr_gps_fix) {
      case SIM_GNSS_NO_FIX: {
        gps_glyph = &logo_gps_sat_no_fix;
      } break;

      case SIM_GNSS_FIX: {
        gps_glyph = &logo_gps_sat_fix;
      } break;
      
      default: {
        ESP_LOGW(DBG_TAG, "Invalid SIM_GnssFix_t received (%d)", curr_gps_fix);
      } break;
    }

    if (gps_glyph != NULL) {
      ESP_LOGI(DBG_TAG, "Drawing GPS glyph");
      for (unsigned int i = 0; i < ((gps_glyph->width * gps_glyph->height * gps_glyph->bytes_per_pixel) - 1); i += gps_glyph->bytes_per_pixel) {
        if (x >= gps_glyph->width + DISP_GPS_GLYPH_X_OFFSET) {
          x = DISP_GPS_GLYPH_X_OFFSET;
          y++;
        }
        uint16_t concat = ((gps_glyph->pixel_data[i+1] << 8) + gps_glyph->pixel_data[i]);
        dispcolor_DrawPixel(x++, y, concat);  
      }
    }
  }
};

void disp_update_gps_sats_in_view(void) {
  if (curr_siv != last_siv || curr_siu != last_siu) {
    ESP_LOGI(DBG_TAG, "Sats info changed <SIV, SIU>: <%d,%d> -> <%d,%d>", last_siv, curr_siv, last_siu, curr_siu);
    last_siv = curr_siv;
    last_siu = curr_siu;

    // Clear display
    dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET, DISP_GPS_BARS_Y_OFFSET, DISP_GPS_BARS_X_OFFSET + 28, DISP_GPS_BARS_Y_OFFSET - 20, BLACK);

    int bars = 0;
    uint16_t color = WHITE;
    if (curr_gps_fix == SIM_GNSS_NO_FIX) {
      color = BLUE;
      if (curr_siv <= 1) {
        bars = 0;
      } else if (curr_siv <= 2) {
        bars = 1;
      } else if (curr_siv <= 5) {
        bars = 2;
      } else if (curr_siv <= 7) {
        bars = 3;
      } else if (curr_siv <= 10) {
        bars = 4;
      } else {
        bars = 5;
      }
    } else {
      if (curr_siu <= 2) {
        bars = 0;
      } else if (curr_siu <= 3) {
        bars = 1;
      } else if (curr_siu <= 5) {
        bars = 2;
      } else if (curr_siu <= 7) {
        bars = 3;
      } else if (curr_siu <= 9) {
        bars = 4;
      } else {
        bars = 5;
      }
    }

    // Empty bars
    dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET, DISP_GPS_BARS_Y_OFFSET - 2, DISP_GPS_BARS_X_OFFSET + 4, DISP_GPS_BARS_Y_OFFSET, color);
    dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 6, DISP_GPS_BARS_Y_OFFSET - 2, DISP_GPS_BARS_X_OFFSET + 10, DISP_GPS_BARS_Y_OFFSET, color);
    dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 12, DISP_GPS_BARS_Y_OFFSET - 2, DISP_GPS_BARS_X_OFFSET + 16, DISP_GPS_BARS_Y_OFFSET, color);
    dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 18, DISP_GPS_BARS_Y_OFFSET - 2, DISP_GPS_BARS_X_OFFSET + 22, DISP_GPS_BARS_Y_OFFSET, color);
    dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 24, DISP_GPS_BARS_Y_OFFSET - 2, DISP_GPS_BARS_X_OFFSET + 28, DISP_GPS_BARS_Y_OFFSET, color);

    switch (bars) {
      case 5: {
        dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 24, DISP_GPS_BARS_Y_OFFSET - 20, DISP_GPS_BARS_X_OFFSET + 28, DISP_GPS_BARS_Y_OFFSET, color);  // bar 5
      }  // fallthrough-intentional

      case 4: {
        dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 18, DISP_GPS_BARS_Y_OFFSET - 16, DISP_GPS_BARS_X_OFFSET + 22, DISP_GPS_BARS_Y_OFFSET, color);  // bar 4
      }  // fallthrough-intentional

      case 3: {
        dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 12, DISP_GPS_BARS_Y_OFFSET - 12, DISP_GPS_BARS_X_OFFSET + 16, DISP_GPS_BARS_Y_OFFSET, color);  // bar 3
      }  // fallthrough-intentional

      case 2: {
        dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET + 6, DISP_GPS_BARS_Y_OFFSET - 8, DISP_GPS_BARS_X_OFFSET + 10, DISP_GPS_BARS_Y_OFFSET, color);  // bar 2
      }  // fallthrough-intentional
      
      case 1: {
        dispcolor_FillRectangle(DISP_GPS_BARS_X_OFFSET, DISP_GPS_BARS_Y_OFFSET - 4, DISP_GPS_BARS_X_OFFSET + 4, DISP_GPS_BARS_Y_OFFSET, color);  // bar 1
      }  // fallthrough-intentional

      case 0: {
        // Nothing to do
      } break;
    
      default: {
        ESP_LOGW(DBG_TAG, "Unknown gps bars: (%d)", bars);
      }
    }
  }
}

void disp_draw_text_fields(void) {
}

void disp_draw_speed_glyph(void) {
  DISP_Number_t const * tens_glyph = NULL;
  DISP_Number_t const * ones_glyph = NULL;

  int x_tens = DISP_SPEED_GLYPH_TENS_X_OFFSET;
  int y_tens = DISP_SPEED_GLYPH_TENS_Y_OFFSET;
  int x_ones = DISP_SPEED_GLYPH_ONES_X_OFFSET;
  int y_ones = DISP_SPEED_GLYPH_ONES_Y_OFFSET;
  uint16_t concat;

  if (last_sog_mph != curr_sog_mph) {
    ESP_LOGI(DBG_TAG, "SOG updated (%d) -> (%d)", last_sog_mph, curr_sog_mph);
    last_sog_mph = curr_sog_mph;

      switch (curr_sog_mph / 10) {
        case 0: {
          tens_glyph = &number_zero;
        } break;

        case 1: {
          tens_glyph = &number_one;
        } break;

        case 2: {
          tens_glyph = &number_two;
        } break;

        case 3: {
          tens_glyph = &number_three;
        } break;

        case 4: {
          tens_glyph = &number_four;
        } break;

        case 5: {
          tens_glyph = &number_five;
        } break;

        case 6: {
          tens_glyph = &number_six;
        } break;

        case 7: {
          tens_glyph = &number_seven;
        } break;

        case 8: {
          tens_glyph = &number_eight;
        } break;

        case 9: {
          tens_glyph = &number_nine;
        } break;

        default: {
          ESP_LOGW(DBG_TAG, "Received bad SOG Tens place!");
        } break;
      }

      switch (curr_sog_mph % 10) {
        case 0: {
          ones_glyph = &number_zero;
        } break;

        case 1: {
          ones_glyph = &number_one;
        } break;

        case 2: {
          ones_glyph = &number_two;
        } break;

        case 3: {
          ones_glyph = &number_three;
        } break;

        case 4: {
          ones_glyph = &number_four;
        } break;

        case 5: {
          ones_glyph = &number_five;
        } break;

        case 6: {
          ones_glyph = &number_six;
        } break;

        case 7: {
          ones_glyph = &number_seven;
        } break;

        case 8: {
          ones_glyph = &number_eight;
        } break;

        case 9: {
          ones_glyph = &number_nine;
        } break;

        default: {
          ESP_LOGW(DBG_TAG, "Received bad SOG Tens place!");
        } break;
      }


    if (tens_glyph != NULL) {
      ESP_LOGI(DBG_TAG, "Drawing Speed 10s glyph");
      for (unsigned int i = 0; i < ((tens_glyph->width * tens_glyph->height * tens_glyph->bytes_per_pixel) - 1); i += tens_glyph->bytes_per_pixel) {
        if (x_tens >= tens_glyph->width + DISP_SPEED_GLYPH_TENS_X_OFFSET) {
          x_tens = DISP_SPEED_GLYPH_TENS_X_OFFSET;
          y_tens++;
        }
        uint16_t concat = ((tens_glyph->pixel_data[i+1] << 8) + tens_glyph->pixel_data[i]);
        dispcolor_DrawPixel(x_tens++, y_tens, concat);  
      }
    }

    if (ones_glyph != NULL) {
      for (unsigned int i = 0; i < ((ones_glyph->width * ones_glyph->height * ones_glyph->bytes_per_pixel) - 1); i += ones_glyph->bytes_per_pixel) {
        if (x_ones >= ones_glyph->width + DISP_SPEED_GLYPH_ONES_X_OFFSET) {
          x_ones = DISP_SPEED_GLYPH_ONES_X_OFFSET;
          y_ones++;
        }
        uint16_t concat = ((ones_glyph->pixel_data[i+1] << 8) + ones_glyph->pixel_data[i]);
        dispcolor_DrawPixel(x_ones++, y_ones, concat);  
      }
    }
  }
}

/**
 * @brief  disp_application_task draws data to the screen at a set interval
 *
 * @param  args: Not used.
 */
void disp_application_task(void *args) {
    // dispcolor_FillScreen(WHITE);
  while (1) {
    disp_draw_carrier_glyph();
    disp_draw_gps_glyph();
    disp_draw_speed_glyph();
    disp_update_cell_signal_strength();
    disp_update_gps_sats_in_view();
    disp_tick++;
    // dispcolor_printf_Bg(((disp_tick * 3) % 240), ((disp_tick * 3) % 240), 1, WHITE, BLACK, "HELLO");
    dispcolor_Update();
    vTaskDelay(pdMS_TO_TICKS(DISP_REFRESH_TASK_PERIOD));
  }
}
