#include <clk.h>
#include <common.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <generic-phy.h>
#include <reset.h>
#include <usb.h>

#ifdef UE11_DEBUG
#define UE11_DEBUG printf
#else
#define UE11_DEBUG(...)
#endif

#ifdef UE11_INFO
#define UE11_INFO printf
#else
#define UE11_INFO(...)
#endif

#define USB_CONTROL_RETRIES         5
#define USB_CONTROL_NAK_RETRY_MS    2
#define USB_CONTROL_NAK_RETRIES     (50/USB_CONTROL_NAK_RETRY_MS)

#define USB_CTRL 0x0
#define USB_CTRL_PHY_DMPULLDOWN 7
#define USB_CTRL_PHY_DMPULLDOWN_SHIFT 7
#define USB_CTRL_PHY_DMPULLDOWN_MASK 0x1

#define USB_CTRL_PHY_DPPULLDOWN 6
#define USB_CTRL_PHY_DPPULLDOWN_SHIFT 6
#define USB_CTRL_PHY_DPPULLDOWN_MASK 0x1

#define USB_CTRL_PHY_TERMSELECT 5
#define USB_CTRL_PHY_TERMSELECT_SHIFT 5
#define USB_CTRL_PHY_TERMSELECT_MASK 0x1

#define USB_CTRL_PHY_XCVRSELECT_SHIFT 3
#define USB_CTRL_PHY_XCVRSELECT_MASK 0x3

#define USB_CTRL_PHY_OPMODE_SHIFT 1
#define USB_CTRL_PHY_OPMODE_MASK 0x3

#define USB_CTRL_TX_FLUSH 1
#define USB_CTRL_TX_FLUSH_SHIFT 1
#define USB_CTRL_TX_FLUSH_MASK 0x1

#define USB_CTRL_ENABLE_SOF 0
#define USB_CTRL_ENABLE_SOF_SHIFT 0
#define USB_CTRL_ENABLE_SOF_MASK 0x1

#define USB_STATUS 0x4
#define USB_STATUS_SOF_TIME_SHIFT 16
#define USB_STATUS_SOF_TIME_MASK 0xffff

#define USB_STATUS_RX_ERROR 2
#define USB_STATUS_RX_ERROR_SHIFT 2
#define USB_STATUS_RX_ERROR_MASK 0x1

#define USB_STATUS_LINESTATE_BITS_SHIFT 0
#define USB_STATUS_LINESTATE_BITS_MASK 0x3

#define USB_IRQ_ACK 0x8
#define USB_IRQ_ACK_DEVICE_DETECT 3
#define USB_IRQ_ACK_DEVICE_DETECT_SHIFT 3
#define USB_IRQ_ACK_DEVICE_DETECT_MASK 0x1

#define USB_IRQ_ACK_ERR 2
#define USB_IRQ_ACK_ERR_SHIFT 2
#define USB_IRQ_ACK_ERR_MASK 0x1

#define USB_IRQ_ACK_DONE 1
#define USB_IRQ_ACK_DONE_SHIFT 1
#define USB_IRQ_ACK_DONE_MASK 0x1

#define USB_IRQ_ACK_SOF 0
#define USB_IRQ_ACK_SOF_SHIFT 0
#define USB_IRQ_ACK_SOF_MASK 0x1

#define USB_IRQ_STS 0xc
#define USB_IRQ_STS_DEVICE_DETECT 3
#define USB_IRQ_STS_DEVICE_DETECT_SHIFT 3
#define USB_IRQ_STS_DEVICE_DETECT_MASK 0x1

#define USB_IRQ_STS_ERR 2
#define USB_IRQ_STS_ERR_SHIFT 2
#define USB_IRQ_STS_ERR_MASK 0x1

#define USB_IRQ_STS_DONE 1
#define USB_IRQ_STS_DONE_SHIFT 1
#define USB_IRQ_STS_DONE_MASK 0x1

#define USB_IRQ_STS_SOF 0
#define USB_IRQ_STS_SOF_SHIFT 0
#define USB_IRQ_STS_SOF_MASK 0x1

#define USB_IRQ_MASK 0x10
#define USB_IRQ_MASK_DEVICE_DETECT 3
#define USB_IRQ_MASK_DEVICE_DETECT_SHIFT 3
#define USB_IRQ_MASK_DEVICE_DETECT_MASK 0x1

#define USB_IRQ_MASK_ERR 2
#define USB_IRQ_MASK_ERR_SHIFT 2
#define USB_IRQ_MASK_ERR_MASK 0x1

#define USB_IRQ_MASK_DONE 1
#define USB_IRQ_MASK_DONE_SHIFT 1
#define USB_IRQ_MASK_DONE_MASK 0x1

#define USB_IRQ_MASK_SOF 0
#define USB_IRQ_MASK_SOF_SHIFT 0
#define USB_IRQ_MASK_SOF_MASK 0x1

#define USB_XFER_DATA 0x14
#define USB_XFER_DATA_TX_LEN_SHIFT 0
#define USB_XFER_DATA_TX_LEN_MASK 0xffff

#define USB_XFER_TOKEN 0x18
#define USB_XFER_TOKEN_START 31
#define USB_XFER_TOKEN_START_SHIFT 31
#define USB_XFER_TOKEN_START_MASK 0x1

#define USB_XFER_TOKEN_IN 30
#define USB_XFER_TOKEN_IN_SHIFT 30
#define USB_XFER_TOKEN_IN_MASK 0x1

#define USB_XFER_TOKEN_ACK 29
#define USB_XFER_TOKEN_ACK_SHIFT 29
#define USB_XFER_TOKEN_ACK_MASK 0x1

#define USB_XFER_TOKEN_PID_DATAX 28
#define USB_XFER_TOKEN_PID_DATAX_SHIFT 28
#define USB_XFER_TOKEN_PID_DATAX_MASK 0x1

#define USB_XFER_TOKEN_PID_BITS_SHIFT 16
#define USB_XFER_TOKEN_PID_BITS_MASK 0xff

#define USB_XFER_TOKEN_DEV_ADDR_SHIFT 9
#define USB_XFER_TOKEN_DEV_ADDR_MASK 0x7f

#define USB_XFER_TOKEN_EP_ADDR_SHIFT 5
#define USB_XFER_TOKEN_EP_ADDR_MASK 0xf

#define USB_RX_STAT 0x1c
#define USB_RX_STAT_START_PEND 31
#define USB_RX_STAT_START_PEND_SHIFT 31
#define USB_RX_STAT_START_PEND_MASK 0x1

#define USB_RX_STAT_CRC_ERR 30
#define USB_RX_STAT_CRC_ERR_SHIFT 30
#define USB_RX_STAT_CRC_ERR_MASK 0x1

#define USB_RX_STAT_RESP_TIMEOUT 29
#define USB_RX_STAT_RESP_TIMEOUT_SHIFT 29
#define USB_RX_STAT_RESP_TIMEOUT_MASK 0x1

#define USB_RX_STAT_IDLE 28
#define USB_RX_STAT_IDLE_SHIFT 28
#define USB_RX_STAT_IDLE_MASK 0x1

#define USB_RX_STAT_RESP_BITS_SHIFT 16
#define USB_RX_STAT_RESP_BITS_MASK 0xff

#define USB_RX_STAT_COUNT_BITS_SHIFT 0
#define USB_RX_STAT_COUNT_BITS_MASK 0xffff

#define USB_WR_DATA 0x20
#define USB_WR_DATA_DATA_SHIFT 0
#define USB_WR_DATA_DATA_MASK 0xff

#define USB_RD_DATA 0x20
#define USB_RD_DATA_DATA_SHIFT 0
#define USB_RD_DATA_DATA_MASK 0xff

#define USB_RES_OK 0
#define USB_RES_NAK -1
#define USB_RES_STALL -2
#define USB_RES_TIMEOUT -3

// USB PID generation macro
#define PID_GENERATE(pid3, pid2, pid1, pid0)                                   \
  ((pid0 << 0) | (pid1 << 1) | (pid2 << 2) | (pid3 << 3) | ((!pid0) << 4) |    \
   ((!pid1) << 5) | ((!pid2) << 6) | ((!pid3) << 7))

// USB PID values
#define PID_OUT PID_GENERATE(0, 0, 0, 1)   // 0xE1
#define PID_IN PID_GENERATE(1, 0, 0, 1)    // 0x69
#define PID_SOF PID_GENERATE(0, 1, 0, 1)   // 0xA5
#define PID_SETUP PID_GENERATE(1, 1, 0, 1) // 0x2D

#define PID_DATA0 PID_GENERATE(0, 0, 1, 1) // 0xC3
#define PID_DATA1 PID_GENERATE(1, 0, 1, 1) // 0x4B

#define PID_ACK PID_GENERATE(0, 0, 1, 0)   // 0xD2
#define PID_NAK PID_GENERATE(1, 0, 1, 0)   // 0x5A
#define PID_STALL PID_GENERATE(1, 1, 1, 0) // 0x1E

static uint32_t _usb_base = 0xBE000000;
static int _usb_fs_device;

static void usbhw_reg_write(uint32_t addr, uint32_t data) {
  *((volatile uint32_t *)(_usb_base + addr)) = data;
}

static uint32_t usbhw_reg_read(uint32_t addr) {
  return *((volatile uint32_t *)(_usb_base + addr));
}

void usbhw_hub_reset(void) {
  uint32_t val;

  UE11_DEBUG("HW: Enter USB bus reset\n");

  // Power-up / SE0
  val = 0;
  val |= (1 << USB_CTRL_PHY_XCVRSELECT_SHIFT);
  val |= (0 << USB_CTRL_PHY_TERMSELECT_SHIFT);
  val |= (0 << USB_CTRL_PHY_OPMODE_SHIFT);
  val |= (1 << USB_CTRL_PHY_DPPULLDOWN_SHIFT);
  val |= (1 << USB_CTRL_PHY_DMPULLDOWN_SHIFT);
  usbhw_reg_write(USB_CTRL, val);
}

void usbhw_hub_enable(int full_speed, int enable_sof) {
  uint32_t val;

  UE11_DEBUG("HW: Enable root hub\n");

  // Host Full Speed
  val = 0;
  val |= (1 << USB_CTRL_PHY_XCVRSELECT_SHIFT);
  val |= (1 << USB_CTRL_PHY_TERMSELECT_SHIFT);
  val |= (0 << USB_CTRL_PHY_OPMODE_SHIFT);
  val |= (1 << USB_CTRL_PHY_DPPULLDOWN_SHIFT);
  val |= (1 << USB_CTRL_PHY_DMPULLDOWN_SHIFT);
  val |= (1 << USB_CTRL_TX_FLUSH_SHIFT);

  // Enable SOF
  if (enable_sof)
    val |= (1 << USB_CTRL_ENABLE_SOF_SHIFT);

  usbhw_reg_write(USB_CTRL, val);
}

int usbhw_hub_device_detected(void) {
  // Get line state
  uint32_t status = usbhw_reg_read(USB_STATUS);
  status >>= USB_STATUS_LINESTATE_BITS_SHIFT;
  status &= USB_STATUS_LINESTATE_BITS_MASK;

  // FS: D+ pulled high
  // LS: D- pulled high
  _usb_fs_device = (status & 1);

  return (status != 0);
}

int usbhw_hub_full_speed_device(void) { return _usb_fs_device; }

void usbhw_timer_sleep(int ms) { udelay(ms * 1000); }

int usbhw_transfer_in(uint8_t pid, int device_addr, int endpoint,
                      uint8_t *response, uint8_t *rx, int rx_length) {
  int l;
  int rx_count;
  uint32_t token = 0;
  uint8_t data;
  uint32_t status;

  UE11_DEBUG("TOKEN: %s", (pid == PID_SETUP)
                          ? "SETUP"
                          : (pid == PID_DATA0)
                                ? "DATA0"
                                : (pid == PID_DATA1)
                                      ? "DATA1"
                                      : (pid == PID_IN) ? "IN" : "OUT");
  UE11_DEBUG("  DEV %d EP %d\n", device_addr, endpoint);

  // No data to send
  usbhw_reg_write(USB_XFER_DATA, 0);

  // Configure transfer
  token = (((uint32_t)pid) << USB_XFER_TOKEN_PID_BITS_SHIFT) |
          (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) |
          (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
  token |= (1 << USB_XFER_TOKEN_START_SHIFT);
  token |= (1 << USB_XFER_TOKEN_IN_SHIFT);
  token |= (1 << USB_XFER_TOKEN_ACK_SHIFT);
  usbhw_reg_write(USB_XFER_TOKEN, token);

  while ((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_START_PEND))
    ;

  // Wait for rx idle
  while (
      !((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_IDLE_SHIFT)))
    ;

  if (status & (1 << USB_RX_STAT_CRC_ERR_SHIFT)) {
    UE11_DEBUG("  CRC ERROR\n");
    UE11_DEBUG("USB: CRC ERROR\n");
    return USB_RES_TIMEOUT;
  }

  if (status & (1 << USB_RX_STAT_RESP_TIMEOUT_SHIFT)) {
    UE11_DEBUG("  TIMEOUT\n");
    UE11_DEBUG("USB: IN timeout\n");
    return USB_RES_TIMEOUT;
  }

  // Check for NAK / STALL
  *response =
      ((status >> USB_RX_STAT_RESP_BITS_SHIFT) & USB_RX_STAT_RESP_BITS_MASK);

  if (*response == PID_NAK) {
    UE11_DEBUG("  NAK\n");
    return USB_RES_NAK;
  } else if (*response == PID_STALL) {
    UE11_DEBUG("  STALL\n");
    UE11_DEBUG("USB: IN STALL\n");
    return USB_RES_STALL;
  }

  // Check CRC is ok
  if (status & (1 << USB_RX_STAT_CRC_ERR_SHIFT)) {
    UE11_DEBUG("  CRC ERR\n");
    UE11_DEBUG("USB: CRC Error\n");

    return USB_RES_STALL;
  }

  // How much data was actually received?
  rx_count =
      ((status >> USB_RX_STAT_COUNT_BITS_SHIFT) & USB_RX_STAT_COUNT_BITS_MASK);

  UE11_DEBUG(" Rx %d (PID=%x):\n", rx_count, *response);

  // Assert that user buffer is big enough for the response.
  // NOTE: It's not critical to do this, but we can't easily check CRCs without
  // reading the whole response into a buffer.
  // Hitting this condition may point towards issues with higher level protocol
  // implementation...
  assert(rx_length >= rx_count);

  for (l = 0; l < rx_count; l++) {
    data = usbhw_reg_read(USB_RD_DATA);
    UE11_DEBUG(" %02x", data);
    rx[l] = data;
  }
  UE11_DEBUG("\n");

  return rx_count;
}

int usbhw_transfer_out(uint8_t pid, int device_addr, int endpoint,
                       int handshake, uint8_t request, uint8_t *tx,
                       int tx_length) {
  int l;
  uint32_t ctrl = 0;
  uint32_t token = 0;
  uint32_t resp;
  uint32_t status;

  UE11_DEBUG("TOKEN: %s", (pid == PID_SETUP)
                          ? "SETUP"
                          : (pid == PID_DATA0)
                                ? "DATA0"
                                : (pid == PID_DATA1)
                                      ? "DATA1"
                                      : (pid == PID_IN) ? "IN" : "OUT");
  UE11_DEBUG("  DEV %d EP %d\n", device_addr, endpoint);

  // Load DATAx transfer into address 0+
  UE11_DEBUG(" Tx:\n %02x", request);
  for (l = 0; l < tx_length; l++) {
    UE11_DEBUG(" %02x", tx[l]);
    usbhw_reg_write(USB_WR_DATA, tx[l]);
  }
  UE11_DEBUG("\n");

  // Transfer data length
  usbhw_reg_write(USB_XFER_DATA, tx_length);

  // Configure transfer for DATAx portion
  ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);

  // Wait for response or timeout
  ctrl |= (handshake ? (1 << USB_XFER_TOKEN_ACK_SHIFT) : 0);

  ctrl |= ((request == PID_DATA1) ? (1 << USB_XFER_TOKEN_PID_DATAX_SHIFT)
                                  : (0 << USB_XFER_TOKEN_PID_DATAX_SHIFT));

  // Setup token details (don't start transfer yet)
  token = (((uint32_t)pid) << USB_XFER_TOKEN_PID_BITS_SHIFT) |
          (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) |
          (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
  usbhw_reg_write(USB_XFER_TOKEN, token | ctrl);

  // Wait for Tx to start
  while ((status = usbhw_reg_read(USB_RX_STAT)) &
         (1 << USB_RX_STAT_START_PEND_SHIFT))
    ;

  // No handshaking? We are done
  if (!handshake)
    return USB_RES_OK;

  // Wait for idle
  while (
      !((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_IDLE_SHIFT)))
    ;

  if (status & (1 << USB_RX_STAT_RESP_TIMEOUT_SHIFT)) {
    UE11_DEBUG("  TIMEOUT\n");
    UE11_DEBUG("USB: OUT timeout\n");
    return USB_RES_TIMEOUT;
  }

  // Check for NAK / STALL
  resp = ((status >> USB_RX_STAT_RESP_BITS_SHIFT) & USB_RX_STAT_RESP_BITS_MASK);
  if (resp == PID_ACK) {
    UE11_DEBUG("  ACK\n");
    return USB_RES_OK;
  } else if (resp == PID_NAK) {
    UE11_DEBUG("  NAK\n");
    return USB_RES_NAK;
  } else if (resp == PID_STALL) {
    UE11_DEBUG("  STALL\n");
    UE11_DEBUG("USB: OUT STALL\n");
    return USB_RES_STALL;
  }

  UE11_DEBUG("USB: Unknown OUT response (%02x)\n", resp);

  // Unknown
  return USB_RES_STALL;
}

int res_to_usb_state(int res) {
  if (res >= 0) {
    return 0;
  } else if (res == USB_RES_NAK) {
    return USB_ST_NAK_REC;
  } else if (res == USB_RES_STALL) {
    return USB_ST_STALLED;
  } else if (res == USB_RES_TIMEOUT) {
    return USB_ST_CRC_ERR;
  }
  return USB_ST_NOT_PROC;
}
int usb_lowlevel_init(int index, enum usb_init_type init, void **controller) {
  UE11_INFO("%s %d %d %p\n", __func__, index, init, controller);
  usbhw_hub_reset();

  usbhw_timer_sleep(11);
  usbhw_hub_enable(1, 0);
  usbhw_timer_sleep(3);

  UE11_DEBUG("HW: Waiting for device insertion\n");
  while (!usbhw_hub_device_detected())
    ;
  UE11_DEBUG("HW: Device detected\n");

  // Enable SOF
  usbhw_hub_enable(usbhw_hub_full_speed_device(), 1);
  usbhw_timer_sleep(3);
  return 0;
}


int submit_int_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
                   int transfer_len, int interval) {
  int res = -1;
  uint8_t rxBuffer[64];
  uint8_t resp;
  int received_len = 0;
  int nak_retry = 0;

  int addr = usb_pipedevice(pipe);
  int endpoint = usb_pipeendpoint(pipe);
  int type = usb_pipetype(pipe);
  int in = usb_pipein(pipe);
  int out = usb_pipeout(pipe);
  int data = usb_pipedata(pipe);
  int pidData = usb_gettoggle(dev, endpoint, out) ? PID_DATA1 : PID_DATA0;

  dev->status = USB_ST_ACTIVE;
  dev->act_len = 0;
  UE11_INFO("%s dev %p pipe %lx @ %x ep %x data %d type %d %s buf %p len %d\n",
            __func__, dev, pipe, addr, endpoint,
            usb_gettoggle(dev, endpoint, out), type, in ? "in" : "out", buffer,
            transfer_len);
  if (in) {
    // in
    while (received_len < transfer_len && nak_retry < USB_CONTROL_NAK_RETRIES) {
      res = usbhw_transfer_in(PID_IN, addr, endpoint, &resp, rxBuffer,
                              sizeof(rxBuffer));
      dev->status = res_to_usb_state(res);
      if (res == USB_RES_NAK) {
        nak_retry++;
        usbhw_timer_sleep(USB_CONTROL_NAK_RETRY_MS);
        continue;
      }
      if (res < 0) {
        goto end;
      }
      memcpy(buffer + received_len, rxBuffer, min(transfer_len - received_len, res));
      received_len += res;
      dev->act_len = received_len;
    }
    res = received_len;
    goto end;
  } else {
    // out
  }

  dev->status = 0;

end:
  UE11_INFO("%s res %d\n", __func__, res);
  return res;
  return -1;
}

int submit_control_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
                       int transfer_len, struct devrequest *setup) {
  int res = -1;
  uint8_t rxBuffer[64];
  uint8_t resp;
  int received_len = 0;
  int nak_retry = 0;

  int addr = usb_pipedevice(pipe);
  int endpoint = usb_pipe_ep_index(pipe);
  int type = usb_pipetype(pipe);
  int in = usb_pipein(pipe);
  int out = usb_pipeout(pipe);
  int data = usb_pipedata(pipe);

  dev->status = USB_ST_ACTIVE;
  dev->act_len = 0;
  UE11_INFO("%s dev %p pipe %lx @ %x ep %x data %d type %d %s buf %p len %d "
            "setup %p\n",
            __func__, dev, pipe, addr, endpoint, data, type, in ? "in" : "out",
            buffer, transfer_len, setup);
  if (setup) {
    UE11_DEBUG(
        "requesttype %x request %x value 0x%04x index 0x%04x length 0x%04x\n",
        setup->requesttype, setup->request, setup->value, setup->index,
        setup->length);

    if (setup) {
      // setup
      res = usbhw_transfer_out(PID_SETUP, addr, endpoint, 1, PID_DATA0,
                               (uint8_t *)setup, sizeof(struct devrequest));
      dev->status = res_to_usb_state(res);
      if (res < 0) {
        goto end;
      }
    }

    if (in) {
      // control read
      while (received_len < transfer_len && nak_retry < USB_CONTROL_NAK_RETRIES) {
        res = usbhw_transfer_in(PID_IN, addr, endpoint, &resp, rxBuffer,
                                sizeof(rxBuffer));
        dev->status = res_to_usb_state(res);
        if (res == USB_RES_NAK) {
          nak_retry++;
          usbhw_timer_sleep(USB_CONTROL_NAK_RETRY_MS);
          continue;
        }
        if (res < 0) {
          goto end;
        }
        memcpy(buffer + received_len, rxBuffer, min(transfer_len - received_len, res));
        received_len += res;
        dev->act_len = received_len;
      }

      if (received_len == transfer_len) {
        // zero length packet
        res = usbhw_transfer_out(PID_OUT, addr, endpoint, 1, PID_DATA1, NULL, 0);
      }
    } else {
      // control write
      // read response
      res = usbhw_transfer_in(PID_IN, addr, endpoint, &resp, NULL, 0);
      dev->status = res_to_usb_state(res);
      if (res < 0) {
        goto end;
      }
    }
    goto end;
  }

  // nothing happens
  dev->status = 0;

end:
  UE11_INFO("%s res %d\n", __func__, res);
  return res;
}

int submit_bulk_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
                    int transfer_len) {
  int res = -1;
  uint8_t rxBuffer[64];
  uint8_t resp;
  int received_len = 0;

  int addr = usb_pipedevice(pipe);
  int endpoint = usb_pipeendpoint(pipe);
  int type = usb_pipetype(pipe);
  int in = usb_pipein(pipe);
  int out = usb_pipeout(pipe);
  int data = usb_pipedata(pipe);
  int pidData = usb_gettoggle(dev, endpoint, out) ? PID_DATA1 : PID_DATA0;

  dev->status = USB_ST_ACTIVE;
  dev->act_len = 0;
  UE11_INFO("%s dev %p pipe %lx @ %x ep %x data %d type %d %s buf %p len %d\n",
            __func__, dev, pipe, addr, endpoint,
            usb_gettoggle(dev, endpoint, out), type, in ? "in" : "out", buffer,
            transfer_len);
  if (in) {
    // in
    while (received_len < transfer_len) {
      res = usbhw_transfer_in(PID_IN, addr, endpoint, &resp, rxBuffer,
                              sizeof(rxBuffer));
      dev->status = res_to_usb_state(res);
      if (res < 0) {
        goto end;
      }
      memcpy(buffer + received_len, rxBuffer, min(transfer_len - received_len, res));
      received_len += res;
      dev->act_len = received_len;
    }
    res = received_len;
    goto end;
  } else  {
    // out
    res = usbhw_transfer_out(PID_OUT, addr, endpoint, 1, pidData, buffer,
                            transfer_len);
    dev->status = res_to_usb_state(res);
    if (res >= 0) {
      dev->act_len = transfer_len;
      usb_dotoggle(dev, endpoint, out);
    }
    goto end;
  }

  dev->status = 0;

end:
  UE11_INFO("%s res %d\n", __func__, res);
  return res;
}

int usb_lowlevel_stop(int index) {
  UE11_INFO("%s\n", __func__);
  return 0;
}
