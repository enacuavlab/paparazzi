
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
    if(amt->request == AMT22_READ_POSITION) {
      uint8_t p0 = amt->trans.input_buf[0];
      uint8_t p1 = amt->trans.input_buf[1];

      amt->position = ((p0 << 8 | p1) & 0x3fff) >> 2;
      // TODO check checkbits
    }
    else if(amt->request == AMT22_READ_TURNS) {
      uint8_t p0 = amt->trans.input_buf[0];
      uint8_t p1 = amt->trans.input_buf[1];
      uint8_t t0 = amt->trans.input_buf[2];
      uint8_t t1 = amt->trans.input_buf[3];

      amt->position = ((p0 << 8 | p1) & 0x3fff) >> 2;
      amt->turns =    ((t0&0x3F) << 8 | t1);
      // TODO check checkbits
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
