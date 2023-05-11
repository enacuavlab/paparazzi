
#include "amt22.h"

//TODO greter divider ?
#ifndef AMT22_SPI_CDIV
#define AMT22_SPI_CDIV SPIDiv256
#endif

void short_sleep(struct spi_transaction *t) {
  (void)t;
  chThdSleepMicroseconds(4);
}


void amt_submit_byte(struct amt22_t *amt, uint8_t c) {
  amt->trans.output_buf[0] = c;
  amt->trans.output_length = 1;
  amt->trans.input_length = 1;
  spi_submit(amt->periph, &amt->trans);
  amt->byte_no++;
}

void amt22_init(struct amt22_t *amt, struct spi_periph *periph, uint8_t slave_idx) {
  // Set up SPI peripheral and transaction
  amt->periph = periph;
  amt->trans.input_buf = amt->spi_input_buf;
  amt->trans.output_buf = amt->spi_output_buf;
  amt->trans.slave_idx = slave_idx;
  amt->trans.select = SPISelectUnselect;
  amt->trans.cpol = SPICpolIdleLow;
  amt->trans.cpha = SPICphaEdge1;
  amt->trans.dss = SPIDss8bit;
  amt->trans.bitorder = SPIMSBFirst;
  amt->trans.cdiv = AMT22_SPI_CDIV;
  amt->trans.before_cb = short_sleep;
  amt->trans.after_cb = NULL;
  amt->trans.status = SPITransDone;
  amt->byte_no = 0;

  amt->request = AMT22_READ_POSITION;
  amt->position = 0;
  amt->turns = 0;
  amt->offset = 0;
  amt->flag_offset = 0;
  amt->angle_rad = 0;
}

void amt22_request(struct amt22_t *amt, enum amt22_request_t request) {
  if(amt->trans.status == SPITransDone) {
    amt->request = request;
    amt->byte_no = 0;       // reset byte number before starting the transaction.
  }
}

void amt22_event(struct amt22_t *amt) {

  // periph busy
  if(amt->trans.status == SPITransPending || amt->trans.status == SPITransRunning) {
    return;
  }

  if(amt->trans.status == SPITransFailed) {
    amt->trans.status = SPITransDone;
    return;
  }

  // spi success, read data
  if(amt->trans.status == SPITransSuccess) {
    // we do the transaction one byte at a time, so store the data in a persistant buffer.
    amt->persistant_buf[amt->byte_no-1] = amt->trans.input_buf[0];
    amt->trans.status = SPITransDone;

    if(amt->byte_no == 2 && (amt->request == AMT22_READ_POSITION || amt->request == AMT22_SET_ZERO)) {
      uint8_t p0 = amt->persistant_buf[0];
      uint8_t p1 = amt->persistant_buf[1];
      uint16_t data = p0 << 8 | p1;
      
      if(amt22_checkbit(data)){
        if(amt->flag_offset == 0){
          amt->offset = (data & 0x3fff) >> 2;
          amt->flag_offset = 1;
        }
        amt->position = ((data & 0x3fff) >> 2) - amt->offset; // 12 bits so shift 2
        amt->angle_rad = amt->position*2*M_PI/4096;
      }
      amt->request = AMT22_NONE;
    }
    else if(amt->byte_no == 4 && amt->request == AMT22_READ_TURNS) {
      uint8_t p0 = amt->persistant_buf[0];
      uint8_t p1 = amt->persistant_buf[1];
      uint8_t t0 = amt->persistant_buf[2];
      uint8_t t1 = amt->persistant_buf[3];
      uint16_t data = p0 << 8 | p1;
      
      if(amt22_checkbit(data)){
        amt->position = (data & 0x3fff) >> 2; // 12 bits so shift 2
      }
      amt->turns =    (t0 << 8 | t1);
      amt->request = AMT22_NONE;
    }
  }

  // spi ready
  if(amt->trans.status == SPITransDone) {
    if(amt->request == AMT22_READ_POSITION) {
      switch(amt->byte_no) {
        case 0:
          amt->trans.select = SPISelect;
          amt_submit_byte(amt, 0x00);
          break;
        case 1:
          amt->trans.select = SPIUnselect;
          amt_submit_byte(amt, 0x00);
          break;
      }
    }
    if(amt->request == AMT22_SET_ZERO) {
      switch(amt->byte_no) {
        case 0:
          amt->trans.select = SPISelect;
          amt_submit_byte(amt, 0x00);
          break;
        case 1:
          amt->trans.select = SPIUnselect;
          amt_submit_byte(amt, 0x70);
          break;
      }
    }
    else if(amt->request == AMT22_READ_TURNS) {
      switch(amt->byte_no) {
        case 0:
          amt->trans.select = SPISelect;
          amt_submit_byte(amt, 0x00);
          break;
        case 1:
          amt->trans.select = SPINoSelect;
          amt_submit_byte(amt, 0xA0);
          break;
        case 2:
          amt->trans.select = SPINoSelect;
          amt_submit_byte(amt, 0x00);
          break;
        case 3:
          amt->trans.select = SPIUnselect;
          amt_submit_byte(amt, 0x00);
          break;
      }

    }
    
  }


}

bool amt22_checkbit(uint16_t data) {
  uint8_t rx_even = (data >> 14) & 0x01;
  uint8_t rx_odd = (data >> 15) & 0x01;

  uint8_t odd = 0;
  uint8_t even = 0;
  for(int i=0; i<7; i++) {
    even ^= (data>>(2*i)) & 0x01;
    odd  ^= (data>>(2*i+1)) & 0x01;
  }
  even = !even;
  odd  = !odd;
  return even == rx_even && odd == rx_odd;
    
}
