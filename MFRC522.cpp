/**************************************************************************/
/*!

  @file    MFRC522.cpp
  @author  Bjarte Johansen
  @licence ljos.mit-license.org

  SPI Driver for MFRC522 NFC/13.56 RFID Transceiver.

  Based on code by Dr.Leong ( WWW.B2CQSHOP.COM ) and
  Miguel Balboa (https://github.com/miguelbalboa/rfid).

 */
/**************************************************************************/

#include <Arduino.h>
#include <MFRC522.h>

/**************************************************************************/
/*!

  @brief Instantiates a new MFRC522 class.

  @param sad    SPI chip select pin (CS/SS/SSEL)
  @param reset  Not reset and power-down pin.

 */
/**************************************************************************/
MFRC522::MFRC522(int sad, int reset) {
  _sad = sad;
  pinMode(_sad, OUTPUT);         // Set digital as OUTPUT to connect it to the RFID /ENABLE pin
  digitalWrite(_sad, LOW);


  _reset = reset;
  pinMode(_reset, OUTPUT);       // Set digital pin, Not Reset and Power-Down
  digitalWrite(_reset, HIGH);

}

/**************************************************************************/
/*!

  @brief   Writes value to a register.

  @param   addr  The address a register.
  @param   val   The value to write to a register.

 */
/**************************************************************************/
void MFRC522::writeToRegister(byte addr, byte val) {
  digitalWrite(_sad, LOW);

  //Address format: 0XXXXXX0
  SPI.transfer((addr<<1)&0x7E);
  SPI.transfer(val);

  digitalWrite(_sad, HIGH);
}

/**************************************************************************/
/*!

  @brief   Reads the value at a register.

  @param   addr  The address a register.

  @returns The byte at the register.

 */
/**************************************************************************/
byte MFRC522::readFromRegister(byte addr) {
  byte val;
  digitalWrite(_sad, LOW);
  SPI.transfer(((addr<<1)&0x7E) | 0x80);
  val =SPI.transfer(0x00);
  digitalWrite(_sad, HIGH);
  return val;
}

/**************************************************************************/
/*!

  @brief   Adds a bitmask to a register.

  @param   addr   The address a register.
  @param   mask  The mask to update the register with.

 */
/**************************************************************************/
void MFRC522::setBitMask(byte addr, byte mask) {
  byte current;
  current = readFromRegister(addr);
  writeToRegister(addr, current | mask);
}

/**************************************************************************/
/*!

  @brief   Removes a bitmask from the register.

  @param   reg   The address a register.
  @param   mask  The mask to update the register with.

 */
/**************************************************************************/
void MFRC522::clearBitMask(byte addr, byte mask) {
  byte current;
  current = readFromRegister(addr);
  writeToRegister(addr, current & (~mask));
}

/**************************************************************************/
/*!

  @brief   Does the setup for the MFRC522.

 */
/**************************************************************************/
void MFRC522::begin() {
  digitalWrite(_sad, HIGH);

  reset();

  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  writeToRegister(TModeReg, 0x8D);		// Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  writeToRegister(TPrescalerReg, 0x3E);         // TModeReg[3..0] + TPrescalerReg
  writeToRegister(TReloadRegL, 30);
  writeToRegister(TReloadRegH, 0);

  writeToRegister(TxAutoReg, 0x40);	        // 100%ASK
  writeToRegister(ModeReg, 0x3D);		// CRC initial value 0x6363

  setBitMask(TxControlReg, 0x03);               // Turn antenna on.
}

/**************************************************************************/
/*!

  @brief   Sends a SOFTRESET command to the MFRC522 chip.

 */
/**************************************************************************/
void MFRC522::reset() {
  writeToRegister(CommandReg, MFRC522_SOFTRESET);
}

/**************************************************************************/
/*!

  @brief   Checks the firmware version of the chip.

  @returns The firmware version of the MFRC522 chip.

 */
/**************************************************************************/
byte MFRC522::getFirmwareVersion() {
  byte response;
  response = readFromRegister(VersionReg);
  return response;
}

/**************************************************************************/
/*!

  @brief   Sends a command to a tag.

  @param   cmd     The command to the MFRC522 to send a command to the tag.
  @param   data    The data that is needed to complete the command.
  @param   dlen    The length of the data.
  @param   result  The result returned by the tag.
  @param   rlen    The number of valid bits in the resulting value.

  @returns Returns the status of the calculation.
           MI_ERR        if something went wrong,
           MI_NOTAGERR   if there was no tag to send the command to.
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int MFRC522::commandTag(byte cmd, byte *data, int dlen, byte *result, int *rlen) {
  int status = MI_ERR;
  byte irqEn = 0x00;
  byte waitIRq = 0x00;
  byte lastBits, n;
  int i;

  switch (cmd) {
  case MFRC522_AUTHENT:
    irqEn = 0x12;
    waitIRq = 0x10;
    break;
  case MFRC522_TRANSCEIVE:
    irqEn = 0x77;
    waitIRq = 0x30;
    break;
  default:
    break;
  }

  writeToRegister(CommIEnReg, irqEn|0x80);	// interrupt request
  clearBitMask(CommIrqReg, 0x80);               // Clear all interrupt requests bits.
  setBitMask(FIFOLevelReg, 0x80);               // FlushBuffer=1, FIFO initialization.

  writeToRegister(CommandReg, MFRC522_IDLE);	// No action, cancel the current command.

  // Write to FIFO
  for (i=0; i < dlen; i++) {
    writeToRegister(FIFODataReg, data[i]);
  }

  // Execute the command.
  writeToRegister(CommandReg, cmd);
  if (cmd == MFRC522_TRANSCEIVE) {
    setBitMask(BitFramingReg, 0x80);		// StartSend=1, transmission of data starts
  }

  // Waiting for the command to complete so we can receive data.
  i = 25; // Max wait time is 25ms.
  do {
    delay(1);
    // CommIRqReg[7..0]
    // Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = readFromRegister(CommIrqReg);
    i--;
  } while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  clearBitMask(BitFramingReg, 0x80);	         // StartSend=0

  if (i != 0) { // Request did not time out.
    if(!(readFromRegister(ErrorReg) & 0x1B)) {   // BufferOvfl Collerr CRCErr ProtocolErr
      status = MI_OK;
      if (n & irqEn & 0x01) {
        status = MI_NOTAGERR;
      }

      if (cmd == MFRC522_TRANSCEIVE) {
        n = readFromRegister(FIFOLevelReg);
        lastBits = readFromRegister(ControlReg) & 0x07;
        if (lastBits) {
          *rlen = (n-1)*8 + lastBits;
        } else {
          *rlen = n*8;
        }

        if (n == 0) {
          n = 1;
        }

        if (n > MAX_LEN) {
          n = MAX_LEN;
        }

        // Reading the recieved data from FIFO.
        for (i=0; i<n; i++) {
          result[i] = readFromRegister(FIFODataReg);
        }
      }
    } else {
      status = MI_ERR;
    }
  }
  return status;
}

/**************************************************************************/
/*!

  @brief   Checks to see if there is a tag in the vicinity.

  @param   mode  The mode we are requsting in.
  @param   type  If we find a tag, this will be the type of that tag.
                 0x4400 = Mifare_UltraLight
                 0x0400 = Mifare_One(S50)
                 0x0200 = Mifare_One(S70)
                 0x0800 = Mifare_Pro(X)
                 0x4403 = Mifare_DESFire

  @returns Returns the status of the request.
           MI_ERR        if something went wrong,
           MI_NOTAGERR   if there was no tag to send the command to.
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int MFRC522::requestTag(byte mode, byte *data) {
  int status, len;
  writeToRegister(BitFramingReg, 0x07);  // TxLastBists = BitFramingReg[2..0]

  data[0] = mode;
  status = commandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);

  if ((status != MI_OK) || (len != 0x10)) {
    status = MI_ERR;
  }

  return status;
}

/**************************************************************************/
/*!

  @brief   Handles collisions that might occur if there are multiple
           tags available.

  @param   serial  The serial nb of the tag.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_NOTAGERR   if there was no tag to send the command to.
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int MFRC522::antiCollision(byte *serial) {
  int status, i, len;
  byte check = 0x00;

  writeToRegister(BitFramingReg, 0x00);	     // TxLastBits = BitFramingReg[2..0]

  serial[0] = MF1_ANTICOLL;
  serial[1] = 0x20;
  status = commandTag(MFRC522_TRANSCEIVE, serial, 2, serial, &len);

  if (status == MI_OK) {
    // The checksum of the tag is the ^ of all the values.
    for (i=0; i < (len/8)-1; i++) { // len is in bits, and we want each byte.
      check ^= serial[i];
    }
    // The checksum should be the same as the one provided from the
    // tag (serial[4]).
    if (check != serial[i]) {
      status = MI_ERR;
    }
  }

  return status;
}

/**************************************************************************/
/*!

  @brief   Calculates the CRC value for some data that should be sent to
           a tag.

  @param   data    The data to calculate the value for.
  @param   len     The length of the data.
  @param   result  The result of the CRC calculation.

 */
/**************************************************************************/
void MFRC522::calculateCRC(byte *data, int len, byte *result) {
  int i;
  byte n;

  clearBitMask(DivIrqReg, 0x04);			// CRCIrq = 0
  setBitMask(FIFOLevelReg, 0x80);			// Clear the FIFO pointer

  //Writing data to the FIFO.
  for (i = 0; i < len; i++) {
    writeToRegister(FIFODataReg, data[i]);
  }
  writeToRegister(CommandReg, MFRC522_CALCCRC);

  // Wait for the CRC calculation to complete.
  i = 0xFF;
  do {
    n = readFromRegister(DivIrqReg);
    i--;
  } while ((i!=0) && !(n&0x04));			//CRCIrq = 1

  // Read the result from the CRC calculation.
  result[0] = readFromRegister(CRCResultRegL);
  result[1] = readFromRegister(CRCResultRegM);
}

/**************************************************************************/
/*!

  @brief   Selects a tag for processing.

  @param   serial  The serial number of the tag that is to be selected.

  @returns The SAK response from the tag.

 */
/**************************************************************************/
byte MFRC522::selectTag(byte *serial) {
  int i, status, len;
  byte sak;
  byte buffer[9];

  buffer[0] = MF1_SELECTTAG;
  buffer[1] = 0x70;
  for (i = 0; i < 5; i++) {
    buffer[i+2] = serial[i];
  }
  calculateCRC(buffer, 7, &buffer[7]);

  status = commandTag(MFRC522_TRANSCEIVE, buffer, 9, buffer, &len);

  if ((status == MI_OK) && (len == 0x18)) {
    sak = buffer[0];
  }
  else {
    sak = 0;
  }

  return sak;
}

/**************************************************************************/
/*!

  @brief   Handles the authentication between the tag and the reader.

  @param   mode    What authentication key to use.
  @param   block   The block that we want to read.
  @param   key     The authentication key.
  @param   serial  The serial of the tag.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int MFRC522::authenticate(byte mode, byte block, byte *key, byte *serial) {
  int i, status, len;
  byte buffer[12];

  //Verify the command block address + sector + password + tag serial number
  buffer[0] = mode;
  buffer[1] = block;
  for (i = 0; i < 6; i++) {
    buffer[i+2] = key[i];
  }
  for (i = 0; i < 4; i++) {
    buffer[i+8] = serial[i];
  }

  status = commandTag(MFRC522_AUTHENT, buffer, 12, buffer, &len);

  if ((status != MI_OK) || (!(readFromRegister(Status2Reg) & 0x08))) {
    status = MI_ERR;
  }

  return status;
}

/**************************************************************************/
/*!

  @brief   Tries to read from the current (authenticated) tag.

  @param   block   The block that we want to read.
  @param   result  The resulting value returned from the tag.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int MFRC522::readFromTag(byte block, byte *result) {
  int status, len;

  result[0] = MF1_READ;
  result[1] = block;
  calculateCRC(result, 2, &result[2]);
  status = commandTag(MFRC522_TRANSCEIVE, result, 4, result, &len);

  if ((status != MI_OK) || (len != 0x90)) {
    status = MI_ERR;
  }

  return status;
}

/**************************************************************************/
/*!

  @brief   Tries to write to a block on the current tag.

  @param   block  The block that we want to write to.
  @param   data   The data that we shoudl write to the block.

  @returns Returns the status of the collision detection.
           MI_ERR        if something went wrong,
           MI_OK         if everything went OK.

 */
/**************************************************************************/
int MFRC522::writeToTag(byte block, byte *data) {
  int status, i, len;
  byte buffer[18];

  buffer[0] = MF1_WRITE;
  buffer[1] = block;
  calculateCRC(buffer, 2, &buffer[2]);
  status = commandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);

  if ((status != MI_OK) || (len != 4) || ((buffer[0] & 0x0F) != 0x0A)) {
    status = MI_ERR;
  }

  if (status == MI_OK) {
    for (i = 0; i < 16; i++) {
      buffer[i] = data[i];
    }
    calculateCRC(buffer, 16, &buffer[16]);
    status = commandTag(MFRC522_TRANSCEIVE, buffer, 18, buffer, &len);

    if ((status != MI_OK) || (len != 4) || ((buffer[0] & 0x0F) != 0x0A)) {
      status = MI_ERR;
    }
  }

  return status;
}

/**************************************************************************/
/*!

  @brief   Sends a halt command to the current tag.

 */
/**************************************************************************/
void MFRC522::haltTag() {
  int status, len;
  byte buffer[4];

  buffer[0] = MF1_HALT;
  buffer[1] = 0;
  calculateCRC(buffer, 2, &buffer[2]);
  clearBitMask(Status2Reg, 0x08); // turn off encryption
  status = commandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);
}
