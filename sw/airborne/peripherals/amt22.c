
#include "amt22.h"

//TODO greter divider ?
#ifndef AMT22_SPI_CDIV
#define AMT22_SPI_CDIV SPIDiv256
#endif


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
  amt->trans.before_cb = NULL;
  amt->trans.after_cb = NULL;
  amt->trans.status = SPITransDone;

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
    if(amt->request == AMT22_READ_POSITION || amt->request == AMT22_SET_ZERO) {
      uint8_t p0 = amt->trans.input_buf[0];
      uint8_t p1 = amt->trans.input_buf[1];
      uint16_t data = p0 << 8 | p1;
      
      if(amt22_checkbit(data)){
        if(amt->flag_offset == 0){
          amt->offset = (data & 0x3fff) >> 2;
          amt->flag_offset = 1;
        }
        amt->position = ((data & 0x3fff) >> 2) - amt->offset; // 12 bits so shift 2
        amt->angle_rad = amt->position*2*M_PI/4096;
      }
      
    }
    else if(amt->request == AMT22_READ_TURNS) {
      uint8_t p0 = amt->trans.input_buf[0];
      uint8_t p1 = amt->trans.input_buf[1];
      uint8_t t0 = amt->trans.input_buf[2];
      uint8_t t1 = amt->trans.input_buf[3];
      uint16_t data = p0 << 8 | p1;
      
      if(amt22_checkbit(data)){
        amt->position = (data & 0x3fff) >> 2; // 12 bits so shift 2
      }
      amt->turns =    (t0 << 8 | t1);
      
    }

    amt->trans.status = SPITransDone;
    amt->request = AMT22_NONE;
  }

  // spi ready
  if(amt->trans.status == SPITransDone) {
    if(amt->request == AMT22_READ_POSITION) {
      amt->trans.output_buf[0] = 0x00;
      amt->trans.output_buf[1] = 0x00;
      amt->trans.output_length = 2;
      amt->trans.input_length = 2;
      spi_submit(amt->periph, &amt->trans);
    }
    if(amt->request == AMT22_SET_ZERO) {
      amt->trans.output_buf[0] = 0x00;
      amt->trans.output_buf[1] = 0x70;
      amt->trans.output_length = 2;
      amt->trans.input_length = 2;
      spi_submit(amt->periph, &amt->trans);
    }
    else if(amt->request == AMT22_READ_TURNS) {
      amt->trans.output_buf[0] = 0x00;
      amt->trans.output_buf[1] = 0xA0;
      amt->trans.output_buf[2] = 0x00;
      amt->trans.output_buf[3] = 0x00;
      amt->trans.output_length = 4;
      amt->trans.input_length = 4;
      spi_submit(amt->periph, &amt->trans);
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
