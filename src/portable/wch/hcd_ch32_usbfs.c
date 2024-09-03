/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Joel Michael
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

// based on hdc_template.c, with some parts inspired by hw/mcu/ch32v20x/EVT/EXAM/USB/USBFS/HOST_*
#include "tusb_option.h"

#if CFG_TUH_ENABLED && defined(TUP_USBIP_WCH_USBFS) && CFG_TUH_WCH_USBIP_USBFS

  #include "host/hcd.h"
  #include "host/usbh.h"

  #include "ch32_usbfs_reg.h"
  #include "ch32v20x.h"

//--------------------------------------------------------------------+
// Private macros, definitions, variables, and functions
//--------------------------------------------------------------------+

  // magic numbers: TX/RX buffer areas (2), 2 buffers each (2), 64 byte buffers
  // TX buffers = dma_buf[0]
  // RX buffers = dma_buf[1]
  #define DMA_BUF_TX 0
  #define DMA_BUF_RX 1

TU_ATTR_ALIGNED(4)
static uint8_t setup_buf[8] = {0};
TU_ATTR_ALIGNED(4)
static uint8_t dma_buf[2][64] = {0};

tusb_speed_t port_speed = 0;

// CH32V doesn't directly expose a frame counter, but does provide a SOF IRQ
// Use the SOF IRQ to increment the frame counter.
// The only user of this appears to be osal_task_delay() when CFG_TUSB_OS == OPT_OS_NONE
static volatile uint32_t frame_number = 0;

// flag indicating SOF IRQ has fired
static volatile bool sof_irq = false;

// original xfer values for IRQ handler to access
static uint8_t xfer_token = 0;
static uint8_t xfer_dev_addr = 0;
static uint8_t xfer_ep_addr = 0;
static uint8_t *xfer_buffer = NULL;
static uint16_t xfer_buflen = 0;
static uint16_t xfer_actual_len = 0;

// IRQ handler will retry a few times
  #define XFER_RETRY_TIMES 10
static volatile uint16_t xfer_retry_count = 0;

void ch32_usbfs_log_status(void) {
  TU_LOG_HEX(3, USBOTG_H_FS->BASE_CTRL);
  TU_LOG_HEX(3, USBOTG_H_FS->INT_EN);
  TU_LOG_HEX(3, USBOTG_H_FS->DEV_ADDR);
  TU_LOG_HEX(3, USBOTG_H_FS->MIS_ST);
  TU_LOG_HEX(3, USBOTG_H_FS->INT_FG);
  TU_LOG_HEX(3, USBOTG_H_FS->INT_ST);
  TU_LOG_HEX(3, USBOTG_H_FS->RX_LEN);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_CTRL);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_EP_MOD);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_RX_DMA);
  TU_LOG_HEX(3, dma_buf[DMA_BUF_RX]);
  TU_LOG_BUF(3, dma_buf[DMA_BUF_RX], 64);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_TX_DMA);
  TU_LOG_HEX(3, dma_buf[DMA_BUF_TX]);
  TU_LOG_BUF(3, dma_buf[DMA_BUF_TX], 64);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_SETUP);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_EP_PID);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_RX_CTRL);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_TX_LEN);
  TU_LOG_HEX(3, USBOTG_H_FS->HOST_TX_CTRL);
}

// update device speed variable
// this can only reliably be detected immediately on connection
void update_device_speed(void) {
  TU_LOG_LOCATION();
  ch32_usbfs_log_status();

  // don't update device speed during port reset
  if ((USBOTG_H_FS->HOST_CTRL & USBFS_UHOST_CTRL_BUS_RESET) == USBFS_UHOST_CTRL_BUS_RESET)
    return;

  TU_LOG_HEX(3, USBOTG_H_FS->MIS_ST);
  if ((USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH) == USBFS_UMS_DEV_ATTACH) {
    // device is attached, set speed based on current DM level
    // also set the USB Host's port speed to match
    if ((USBOTG_H_FS->MIS_ST & USBFS_UMS_DM_LEVEL) == USBFS_UMS_DM_LEVEL) {
      port_speed = TUSB_SPEED_LOW;
      USBOTG_H_FS->BASE_CTRL |= USBFS_CTRL_LOW_SPEED;
      USBOTG_H_FS->HOST_CTRL |= USBFS_UHOST_CTRL_LOW_SPEED;
    } else {
      port_speed = TUSB_SPEED_FULL;
      USBOTG_H_FS->BASE_CTRL &= ~(USBFS_CTRL_LOW_SPEED);
      USBOTG_H_FS->HOST_CTRL &= ~(USBFS_UHOST_CTRL_LOW_SPEED);
    }
  } else {
    port_speed = TUSB_SPEED_INVALID;
  }
  TU_LOG(3, "Device Speed: %d\r\n", port_speed);
}

bool start_xfer(uint8_t token, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen) {
  TU_LOG(3, "Start USBH xfer\r\n");
  TU_LOG_LOCATION();
  ch32_usbfs_log_status();

  if (xfer_retry_count >= XFER_RETRY_TIMES)
    return false;

  // poll-wait until current xfer is complete
  while ((USBOTG_H_FS->INT_FG & USBFS_INT_FG_SIE_FREE) == 0);

  // clear xfer IRQ because we use RB_UC_INT_BUSY
  USBOTG_H_FS->INT_FG = USBFS_INT_FG_TRANSFER;

  xfer_token = token;
  xfer_dev_addr = dev_addr;
  xfer_ep_addr = ep_addr;
  xfer_buffer = buffer;
  xfer_buflen = buflen;
  xfer_actual_len = TU_MIN(buflen, 64);

  TU_LOG_HEX(3, token);
  TU_LOG_HEX(3, dev_addr);
  TU_LOG_HEX(3, ep_addr);
  TU_LOG_HEX(3, buflen);
  TU_LOG_HEX(3, buffer);
  TU_LOG_BUF(3, buffer, buflen);
  TU_LOG_INT(3, xfer_actual_len);

  // set device address
  USBOTG_H_FS->DEV_ADDR = dev_addr;

  // check if low-speed preamble is needed (i.e. low-speed device attached to full-speed hub)
  if (tuh_speed_get(dev_addr) == port_speed)
    USBOTG_H_FS->HOST_SETUP &= ~(USBFS_UH_PRE_PID_EN);
  else
    USBOTG_H_FS->HOST_SETUP |= USBFS_UH_PRE_PID_EN;

  // TODO
  // - figure out TX/RX data0/1 toggles
  // - special ISO transfer ack handling?
  switch (token) {
    case HOST_TOKEN_IN: {
      TU_LOG_LOCATION();
      // use aligned DMA buffer if unaligned
      // copy back after RX completes
      break;
    }
    case HOST_TOKEN_SETUP: {
      // intentional fall-through
    }
    case HOST_TOKEN_OUT: {
      TU_LOG_LOCATION();
      // OUT and SETUP are identical except for the token
      memcpy(dma_buf[DMA_BUF_TX], buffer, xfer_actual_len);
      TU_LOG_BUF(3, dma_buf[DMA_BUF_TX], xfer_actual_len);
      USBOTG_H_FS->HOST_TX_LEN = xfer_actual_len;
      TU_LOG_LOCATION();
      break;
    }
    case HOST_TOKEN_SOF:
    default: {
      TU_LOG_LOCATION();
      // SOF should be being internally generated by the host
      // and anything else is an error...
      return false;
    }
  }

  // enable auto-toggle data0/1
  USBOTG_H_FS->HOST_TX_CTRL |= USBFS_UH_T_AUTO_TOG;
  USBOTG_H_FS->HOST_RX_CTRL |= USBFS_UH_R_AUTO_TOG;

  // set frame type token (IN, OUT, SOF, or SETUP) and endpoint number
  // setting EP_PID starts the xfer!
  TU_LOG_LOCATION();
  ch32_usbfs_log_status();
  USBOTG_H_FS->HOST_EP_PID = token | tu_edpt_number(ep_addr);
  TU_LOG_LOCATION();

  // clear xfer IRQ because we use RB_UC_INT_BUSY
  USBOTG_H_FS->INT_FG = USBFS_INT_FG_TRANSFER;
  TU_LOG_LOCATION();

  TU_LOG(3, "LETS FUCKEN GOOOOO\r\n");
  return true;
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

// optional hcd configuration, called by tuh_configure()
bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void *cfg_param) {
  (void) rhport;
  (void) cfg_id;
  (void) cfg_param;
  TU_LOG_LOCATION();
  return false;
}

// Initialize controller to host mode
bool hcd_init(uint8_t rhport) {
  (void) rhport;
  TU_LOG_LOCATION();
  ch32_usbfs_log_status();

  // init registers
  USBOTG_H_FS->BASE_CTRL = USBFS_CTRL_RESET_SIE | USBFS_CTRL_CLR_ALL;
  for (volatile uint16_t i = 0; i < 65535; i++);// micro-sleep
  USBOTG_H_FS->BASE_CTRL = USBFS_CTRL_HOST_MODE;
  USBOTG_H_FS->HOST_CTRL = USBFS_UHOST_CTRL_PORT_EN;
  for (volatile uint16_t i = 0; i < 65535; i++);// micro-sleep
  update_device_speed();

  USBOTG_H_FS->HOST_TX_DMA = (uint32_t) dma_buf[DMA_BUF_TX];
  USBOTG_H_FS->HOST_RX_DMA = (uint32_t) dma_buf[DMA_BUF_RX];
  USBOTG_H_FS->BASE_CTRL |= USBFS_CTRL_INT_BUSY | USBFS_CTRL_DMA_EN;

  USBOTG_H_FS->HOST_EP_MOD = USBFS_UH_EP_TX_EN | USBFS_UH_EP_RX_EN;
  USBOTG_H_FS->HOST_SETUP = USBFS_UH_SOF_EN;
  USBOTG_H_FS->DEV_ADDR = 0x00;

  USBOTG_H_FS->HOST_TX_LEN = 0;
  USBOTG_H_FS->HOST_EP_MOD = USBFS_UH_EP_TX_EN | USBFS_UH_EP_RX_EN;
  USBOTG_H_FS->HOST_RX_CTRL = 0;
  USBOTG_H_FS->HOST_TX_CTRL = 0;

  if ((USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH) == USBFS_UMS_DEV_ATTACH) {
    hcd_event_device_attach(rhport, false);
  }

  USBOTG_H_FS->INT_FG = 0xFF;
  USBOTG_H_FS->INT_EN = USBFS_INT_EN_DETECT | USBFS_INT_EN_TRANSFER | USBFS_INT_EN_HST_SOF;

  TU_LOG_LOCATION();
  ch32_usbfs_log_status();

  return true;
}

// Interrupt Handler
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  (void) rhport;
  (void) in_isr;

  // detect IRQ
  if ((USBOTG_H_FS->INT_FG & USBFS_INT_FG_DETECT) == USBFS_INT_FG_DETECT) {
    TU_LOG(3, "USBFS DETECT IRQ\r\n");

    // always re-enable host port on detect IRQ
    USBOTG_H_FS->HOST_CTRL |= USBFS_UHOST_CTRL_PORT_EN;

    // don't process insert/remove during reset
    if ((USBOTG_H_FS->HOST_CTRL & USBFS_UHOST_CTRL_BUS_RESET) != USBFS_UHOST_CTRL_BUS_RESET) {
      update_device_speed();
      if ((USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH) == USBFS_UMS_DEV_ATTACH) {
        hcd_event_device_attach(rhport, in_isr);
      } else {
        hcd_event_device_remove(rhport, in_isr);
      }
    }
    USBOTG_H_FS->INT_FG = USBFS_INT_FG_DETECT;
  }

  // transfer IRQ
  if ((USBOTG_H_FS->INT_FG & USBFS_INT_FG_TRANSFER) == USBFS_INT_FG_TRANSFER) {
    TU_LOG_LOCATION();
    TU_LOG(3, "USBFS XFER IRQ\r\n");
    TU_LOG_LOCATION();
    ch32_usbfs_log_status();

    // clear HOST_EP_PID will end the xfer
    USBOTG_H_FS->HOST_EP_PID = 0;

    // determine success status of xfer
    // logic based on ch32v20x_usbfs_host.c
    bool xfer_success = false;
    bool xfer_complete = false;
    if ((USBOTG_H_FS->INT_ST & USBFS_INT_ST_TOG_OK) == USBFS_INT_ST_TOG_OK) {
      TU_LOG(3, "USBFS XFER IRQ: USBFS_INT_ST_TOG_OK\r\n");
      switch (xfer_token) {
        case HOST_TOKEN_SETUP: {
          // force data token to data1 for both TX and RX
          USBOTG_H_FS->HOST_TX_CTRL = (USBFS_UH_T_AUTO_TOG | USBFS_UH_T_TOG);
          USBOTG_H_FS->HOST_RX_CTRL = (USBFS_UH_R_AUTO_TOG | USBFS_UH_R_TOG);
          TU_ATTR_FALLTHROUGH;
          // intentional fall-through
        }
        case HOST_TOKEN_OUT: {
          // clear transmit length
          xfer_actual_len = USBOTG_H_FS->HOST_TX_LEN;
          USBOTG_H_FS->HOST_TX_LEN = 0;
          break;
        }
        case HOST_TOKEN_IN: {
          // copy RX DMA buffer back to memory
          memcpy(xfer_buffer, dma_buf[DMA_BUF_RX], xfer_actual_len);
          xfer_actual_len = USBOTG_H_FS->RX_LEN;
          break;
        }
        default: {
          // this shouldn't happen!
          break;
        }
      }

      xfer_success = true;
      xfer_complete = true;
      hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_SUCCESS, true);
    } else {
      if ((USBOTG_H_FS->INT_ST & USBFS_INT_ST_MASK_UIS_H_RES(HOST_PID_STALL)) == HOST_PID_STALL) {
        TU_LOG(3, "USBFS XFER IRQ: HOST_PID_STALL\r\n");
        xfer_success = false;
        xfer_retry_count = XFER_RETRY_TIMES;
        xfer_complete = true;
        hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_STALLED, true);
      } else
        switch (xfer_token) {
          case HOST_TOKEN_SETUP:
          case HOST_TOKEN_OUT: {
            if ((USBOTG_H_FS->INT_ST & 0x0F) != 0) {
              // FIXME - allow NAK of ZLP?!
              if ((USBOTG_H_FS->INT_ST & USBFS_INT_ST_MASK_UIS_H_RES(HOST_PID_NAK)) == HOST_PID_NAK && xfer_actual_len == 0) {
                TU_LOG(3, "USBFS XFER IRQ: NAK of ZLP, accepting anyway\r\n");
                xfer_success = true;
                xfer_complete = true;
                hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_SUCCESS, true);
              } else {
                TU_LOG(3, "USBFS XFER IRQ: OUT status set\r\n");
                xfer_success = false;
                xfer_retry_count = XFER_RETRY_TIMES;
                xfer_complete = true;
                hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_FAILED, true);
              }
            }
            break;
          }
          case HOST_TOKEN_IN: {
            if ((USBOTG_H_FS->INT_ST & 0x0F) == HOST_PID_DATA0 || (USBOTG_H_FS->INT_ST & 0x0F) == HOST_PID_DATA1) {
              break;
            }
            if ((USBOTG_H_FS->INT_ST & 0x0F) != 0) {
              TU_LOG(3, "USBFS XFER IRQ: IN status set\r\n");
              xfer_success = false;
              xfer_retry_count = XFER_RETRY_TIMES;
              xfer_complete = true;
              hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_FAILED, true);
            }
            break;
          }
          default: {
            TU_LOG(3, "USBFS XFER IRQ: invalid token\r\n");
            xfer_success = false;
            xfer_retry_count = XFER_RETRY_TIMES;
            xfer_complete = true;
            hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_INVALID, true);
            break;
          }
        }
    }
    // retry if needed
    if (xfer_complete == false && xfer_success == false && xfer_retry_count < XFER_RETRY_TIMES) {
      xfer_retry_count++;
      TU_LOG(2, "CH32 USBFS Host: xfer retry attempt %d\r\n", xfer_retry_count);
      start_xfer(xfer_token, xfer_dev_addr, xfer_ep_addr, xfer_buffer, xfer_buflen);
    } else if (xfer_complete == false && xfer_success == false && xfer_retry_count >= XFER_RETRY_TIMES) {
      TU_LOG(2, "CH32 USBFS Host: xfer retry timed out\r\n");
      hcd_event_xfer_complete(xfer_dev_addr, xfer_ep_addr, xfer_actual_len, XFER_RESULT_TIMEOUT, true);
    }
  }

  // SOF IRQ
  if ((USBOTG_H_FS->INT_FG & USBFS_INT_FG_HST_SOF) == USBFS_INT_FG_HST_SOF) {
    frame_number++;
    sof_irq = true;
    USBOTG_H_FS->INT_FG = USBFS_INT_FG_HST_SOF;
  }

  // clear all remaining IRQs
  USBOTG_H_FS->INT_FG = 0xFF;
}

// Enable USB interrupt
void hcd_int_enable(uint8_t rhport) {
  (void) rhport;
  NVIC_EnableIRQ(USBFS_IRQn);
  NVIC_EnableIRQ(USBFSWakeUp_IRQn);
}

// Disable USB interrupt
void hcd_int_disable(uint8_t rhport) {
  (void) rhport;
  NVIC_DisableIRQ(USBFS_IRQn);
  NVIC_DisableIRQ(USBFSWakeUp_IRQn);
}

// Get frame number (1ms)
uint32_t hcd_frame_number(uint8_t rhport) {
  (void) rhport;
  //TU_LOG_LOCATION();
  return frame_number;
}

//--------------------------------------------------------------------+
// Port API
//--------------------------------------------------------------------+

// Get the current connect status of roothub port
bool hcd_port_connect_status(uint8_t rhport) {
  (void) rhport;
  if ((USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH) == USBFS_UMS_DEV_ATTACH)
    return true;
  return false;
}

// Reset USB bus on the port. Return immediately, bus reset sequence may not be complete.
// Some port would require hcd_port_reset_end() to be invoked after 10ms to complete the reset sequence.
void hcd_port_reset(uint8_t rhport) {
  (void) rhport;
  // mask attach IRQ
  USBOTG_H_FS->INT_EN &= ~(USBFS_INT_EN_DETECT);
  // clear all pending IRQs
  USBOTG_H_FS->INT_FG = 0xFF;
  // reset bus
  USBOTG_H_FS->HOST_CTRL |= USBFS_UHOST_CTRL_BUS_RESET;

  for (volatile uint16_t i = 0; i < 65535; i++);// micro-sleep
}

// Complete bus reset sequence, may be required by some controllers
void hcd_port_reset_end(uint8_t rhport) {
  (void) rhport;
  // clear reset
  USBOTG_H_FS->HOST_CTRL &= ~(USBFS_UHOST_CTRL_BUS_RESET);
  // re-enable host port after reset
  USBOTG_H_FS->HOST_CTRL |= USBFS_UHOST_CTRL_PORT_EN;

  for (volatile uint16_t i = 0; i < 65535; i++);// micro-sleep

  // clear all pending IRQs
  USBOTG_H_FS->INT_FG = 0xFF;
  // enable attach IRQ
  USBOTG_H_FS->INT_EN |= USBFS_INT_EN_DETECT;
}

// Get port link speed
tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  (void) rhport;
  return port_speed;
}

// HCD closes all opened endpoints belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
  TU_LOG_LOCATION();
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+

// Open an endpoint
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const *ep_desc) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_desc;
  TU_LOG_LOCATION();
  TU_LOG_HEX(3, dev_addr);
  TU_LOG_HEX(3, ep_desc);
  // FIXME - does this need to be used?
  return true;
}

// Submit a transfer, when complete hcd_event_xfer_complete() must be invoked
bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  (void) buffer;
  (void) buflen;
  TU_LOG_LOCATION();
  xfer_retry_count = 0;
  if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
    return start_xfer(HOST_TOKEN_IN, dev_addr, ep_addr, buffer, buflen);
  return start_xfer(HOST_TOKEN_OUT, dev_addr, ep_addr, buffer, buflen);
}

// Abort a queued transfer. Note: it can only abort transfer that has not been started
// Return true if a queued transfer is aborted, false if there is no transfer to abort
bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  TU_LOG_LOCATION();
  xfer_retry_count = XFER_RETRY_TIMES;
  return true;
}

// Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8]) {
  (void) rhport;
  (void) dev_addr;
  (void) setup_packet;
  TU_LOG_LOCATION();
  xfer_retry_count = 0;
  memcpy(setup_buf, setup_packet, 8);

  // reset {TX,RX}_CTRL to data0
  USBOTG_H_FS->HOST_TX_CTRL = USBFS_UH_T_AUTO_TOG;
  USBOTG_H_FS->HOST_RX_CTRL = USBFS_UH_R_AUTO_TOG;

  return start_xfer(HOST_TOKEN_SETUP, dev_addr, 0, setup_buf, 8);
}

// clear stall, data toggle is also reset to DATA0
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  TU_LOG_LOCATION();
  return false;
}

#endif
