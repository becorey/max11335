"""
`max11335`
Refer to the MAX11335-MAX11340 datasheet for further info

* Author: Corey Berman

"""

import machine
import time
import struct


class ADC_MAX11335:
    def __init__(self, pins, channels,
                 reference = 'SINGLE_ENDED',
                 scanControl = 'STANDARD_INT',
                 spiBus = 2, debug = False):
        print('init ADC_MAX11335')
        
        self.pins = pins
        self.channels = channels
        self.reference = reference # SINGLE_ENDED or DIFFERENTIAL
        self.scanControl = scanControl
        self.spiBus = spiBus
        self.debug = debug
        
        self.nChannels = len(self.channels)
        
        #bufferLength = 2 * self.nChannels
        bufferLength = 2 # 2 bytes, 8 bit each, to carry 16 bit data words
        self.writeBuf = bytearray(bufferLength)
        self.readBuf = bytearray(bufferLength)
        
        
        # For SPI/QSPI, ensure the CPU serial interface runs in master
        # mode to generate the serial clock signal. Select the SCLK
        # frequency of 8MHz or less, and set clock polarity (CPOL)
        # and phase (CPHA) in the control registers to the same
        # value. The MAX11335–MAX11340 operate with SCLK
        # idling high, and thus operate with CPOL = CPHA = 1
        self.spi = machine.SPI(
            self.spiBus, baudrate = int(8e6),
            polarity = 1, phase = 1,
            bits = 16, firstbit = machine.SPI.MSB,
            sck = machine.Pin(pins['CLK']),
            mosi = machine.Pin(pins['MOSI']),
            miso = machine.Pin(pins['MISO'])
        )
        
        
        self.csPin = machine.Pin(self.pins['CS_ADC'],
                            mode = machine.Pin.OUT, pull = machine.Pin.PULL_UP)
        self.csPin.value(1)
        
        self.cnvstPin = machine.Pin(self.pins['CNVST'],
                            mode = machine.Pin.OUT, pull = machine.Pin.PULL_UP)
        self.cnvstPin.value(1)
        
        self.eocPin = machine.Pin(self.pins['EOC'],
                            mode = machine.Pin.IN, pull = machine.Pin.PULL_UP)
        self.eocPin.irq(handler = self.conversionComplete, trigger = machine.Pin.IRQ_FALLING)

        echoSuccess = self.verifyEcho()
        if not echoSuccess:
            raise Exception('failed echo test')

        self.setupRegisters()
        
        return
    
    def setupRegisters(self):
        
        # Applications Information, pg 31
        # 1) Configure the ADC (set the MSB on DIN to 1).
        self.configRegister(self.reference, avgOn = 1, nAvg = 4)
        
        time.sleep_us(2)
        self.unipolarRegister()
        time.sleep_us(2)
        self.bipolarRegister()
        
        if self.reference == 'DIFFERENTIAL':
            time.sleep_us(2)
            self.rangeRegister()
        
        if self.scanControl in ['CUSTOM_INT', 'CUSTOM_EXT']:
            time.sleep_us(2)
            self.customScanRegister()
        
        # 2) Program ADC mode control (set the MSB on DIN to 0)
        #    to begin the conversion process or to control power
        #    management features.
        time.sleep_us(2)
        self.modeControl(self.scanControl, channelSelect = 0, reset = 'ALL')
        
        if self.scanControl in ['STANDARD_INT', 'CUSTOM_INT']:
            self.modeControlBit = self.modeControl(self.scanControl,
                                                   channelSelect = self.nChannels - 1,
                                                   reset = 'NONE')
        
        return
    
    def configRegister(self, refSel = 'SINGLE_ENDED',
                          avgOn = 0, nAvg = 0,
                          nScan = 0, staticPowerDownMode = 'NORMAL', echo = 0):
        print('configRegister')
        
        if refSel == 'SINGLE_ENDED':
            referenceBit = 0b0
        elif refSel == 'DIFFERENTIAL':
            referenceBit = 0b1
        else:
            raise ValueError('refSel ', refSel, ' not valid')
        
        if avgOn not in [0, 1]:
            raise ValueError('avgOn ', avgOn, 'not valid')
        
        if avgOn == 0:
            nAvgBit = 0b00
        elif nAvg == 4:
            nAvgBit = 0b00
        elif nAvg == 8:
            nAvgBit = 0b01
        elif nAvg == 16:
            nAvgBit = 0b10
        elif nAvg == 32:
            nAvgBit = 0b11
        else:
            raise ValueError('nAvg ', nAvg, ' not valid')
        
        if nScan not in [0, 1, 2, 3]:
            raise ValueError('nScan ', nScan, ' not valid')
        
        if staticPowerDownMode == 'NORMAL':
            spmBit = 0b00
        elif staticPowerDownMode == 'FULL_SHUTDOWN':
            spmBit = 0b01
        elif staticPowerDownMode == 'PARTIAL_SHUTDOWN':
            spmBit = 0b10
        else:
            raise ValueError('staticPowerDownMode ', staticPowerDownMode, ' not valid')
        
        if echo not in [0, 1]:
            raise ValueError('echo ', echo, ' not valid')
        
        din = (
            0b10000 << 11 |
            referenceBit << 10 |
            avgOn << 9 |
            nAvgBit << 7 |
            nScan << 5 |
            spmBit << 3 |
            echo << 2 |
            0b00            
        )
        
        self.spiWriteReadBin(din)
        
        return din
    
    def modeControl(self, scanControl, channelSelect,
                       reset = 'NONE', powerMode = 'NORMAL',
                       channelAddress = 1, switchConversion = 0, send = True):
        """
        Set the ADC Mode Control Register
        """
        print('modeControl')
        
        if scanControl != 'NULL':
            self.scanControl = scanControl
            

        if scanControl == 'NULL':
            scanControlBit = 0b0000
        elif scanControl == 'MANUAL':
            scanControlBit = 0b0001
        elif scanControl == 'REPEAT':
            scanControlBit = 0b0010
        elif scanControl == 'STANDARD_INT':
            scanControlBit = 0b0011
        elif scanControl == 'STANDARD_EXT':
            scanControlBit = 0b0100
        elif scanControl == 'UPPER_INT':
            scanControlBit = 0b0101
        elif scanControl == 'UPPER_EXT':
            scanControlBit = 0b0110
        elif scanControl == 'CUSTOM_INT':
            scanControlBit = 0b0111
        elif scanControl == 'CUSTOM_EXT':
            scanControlBit = 0b1000
        else:
            raise ValueError('scanControl ', scanControl, ' not implemented')
        
        if channelSelect < 0 or channelSelect > 3:
            raise ValueError('channelSelect ', channelSelect, ' out of range 0-3')
        
        if reset == 'NONE':
            resetBit = 0b00
        elif reset == 'FIFO':
            resetBit = 0b01
        elif reset == 'ALL':
            resetBit = 0b10
        else:
            raise ValueError('reset ', reset, ' not valid')
        
        if powerMode == 'NORMAL':
            powerModeBit = 0b00
        elif powerMode == 'AUTOSHUTDOWN':
            powerModeBit = 0b01
        elif powerMode == 'AUTOSTANDBY':
            powerModeBit = 0b10
            
        if channelAddress not in [0, 1]:
            raise ValueError('channelAddress ', channelAddress, ' not valid')
        
        if switchConversion not in [0, 1]:
            raise ValueError('switchConversion ', switchConversion, ' not valid')
        
        din = (
            0b0 << 15 |
            scanControlBit << 11 |
            channelSelect << 7 |
            resetBit << 5 |
            powerModeBit << 3 |
            channelAddress << 2 |
            switchConversion << 1 |
            0b0
        )
        
        if send:
            self.spiWriteReadBin(din)
        
        return din
    
    def rangeRegister(self):
        """
        RANGE Settings Only Applies to Bipolar Fully Differential
        Analog Input Configurations
        """
        print('rangeRegister')
        
        din = (
            0b010011 << 11 |
            0b0 << 10 |
            0b0 << 9 |
            0b0 << 8 |
            0b0 << 7 |
            0b0 << 6 |
            0b0 << 5 |
            0b0 << 4 |
            0b0 << 3 |
            0b000
        )
        
        self.spiWriteReadBin(din)
        
        return din
        
    
    def unipolarRegister(self, uch = [0,0,0,0,0,0,0,0], pseudodifferentialCommon = 0):
        print('unipolarRegister')
        
        if pseudodifferentialCommon not in [0, 1]:
            raise ValueError('pseudodifferentialCommon ', pseudodifferentialCommon, ' not valid')
        
        din = (
            0b10001 << 11 |
            uch[0] << 10 |
            uch[1] << 9 |
            uch[2] << 8 |
            uch[3] << 7 |
            uch[4] << 6 |
            uch[5] << 5 |
            uch[6] << 4 |
            uch[7] << 3 |
            pseudodifferentialCommon << 2 |
            0b00
        )
        
        self.spiWriteReadBin(din)
        
        return din
    
    def bipolarRegister(self, bch = [0,0,0,0,0,0,0,0]):
        print('bipolarRegister')
        
        din = (
            0b10010 << 11 |
            bch[0] << 10 |
            bch[1] << 9 |
            bch[2] << 8 |
            bch[3] << 7 |
            bch[4] << 6 |
            bch[5] << 5 |
            bch[6] << 4 |
            bch[7] << 3 |
            0b000
        )
        
        self.spiWriteReadBin(din)
        
        return din
    
    def customScanRegister(self, chscan = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
        print('customScanRegister')
        
        for chscani in chscan:
            if chscani not in [0, 1]:
                raise ValueError('chscan ', chscan, ' not valid')
                return
        
        din = (
            0b10100 << 11 |
            chscan[15] << 10 |
            chscan[14] << 9 |
            chscan[13] << 8 |
            chscan[12] << 7 |
            chscan[11] << 6 |
            chscan[10] << 5 |
            chscan[9] << 4 |
            chscan[8] << 3 |
            0b000
        )
        
        self.spiWriteReadBin(din)
        
        din = (
            0b10101 << 11 |
            chscan[7] << 10 |
            chscan[6] << 9 |
            chscan[5] << 8 |
            chscan[4] << 7 |
            chscan[3] << 6 |
            chscan[2] << 5 |
            chscan[1] << 4 |
            chscan[0] << 3 |
            0b000
        )
        
        self.spiWriteReadBin(din)
        
        return din
    
    def sampleSetRegister(self, sequenceLength = 1):
        print('sampleSetRegister')
        
        if sequenceLength < 1 or sequenceLength > 256:
            raise ValueError('sequenceLength ', sequenceLength, ' not valid')
        
        seqLengthBit = sequenceLength - 1
        
        din = (
            0b10110 << 11 |
            seqLengthBit << 3 |
            0b000
        )
        
        self.spiWriteReadBin(din)       
        
        return din
    
    def verifyEcho(self):
        print('verifyEcho')
        
        for i in range(3):
            din = self.configRegister(refSel = 'SINGLE_ENDED',
                avgOn = 0, nAvg = 0,
                nScan = 0, staticPowerDownMode = 'NORMAL', echo = 1)
        
        # t = n-1, turn on echo
        # t = n, send config data
        # t = n+1, receive echo config data
        doutBuf = self.readBuf
        dout = struct.unpack('>H', doutBuf)
        dout = list(dout)
        print('din =', din, '  dout =', dout)
        if din == dout[0]:
            print('echo success')
            return True
        else:
            print('echo failed')
            return False
        
        return
    
    def conversionStart(self, dur = 0):
        """
        Internal Clock.
        The wake-up, acquisition, conversion, and shutdown
        sequences are initiated through CNVST and are performed
        automatically using the internal oscillator. Results
        are added to the internal FIFO.
        With CS high, initiate a scan by setting CNVST low for
        at least 5ns before pulling it high (Figure 6). Then, the
        MAX11335–MAX11340 wake up, scan all requested
        channels, store the results in the FIFO, and shut down.
        After the scan is complete, EOC is pulled low and the
        results are available in the FIFO. Wait until EOC goes
        low before pulling CS low to communicate with the serial
        interface. EOC stays low until CS or CNVST is pulled low
        again. Do not initiate a second CNVST before EOC goes
        low; otherwise, the FIFO may become corrupted.
        """
        print('conversionStart')
        self.csPin.value(1)
        self.cnvstPin.value(0)
        if dur > 0:
            time.sleep_us(dur) #  > 5 ns
        self.cnvstPin.value(1)
        
        return
    
    def conversionComplete(self, pin):
        """
        The serial data output,
        DOUT, delivers the conversion results and is clocked out
        by the falling edge of SCLK. DOUT is a 16-bit data word
        containing a 4-bit channel address, followed by a 12-bit
        conversion result led by the MSB when CHAN_ID is set
        to 1 in the ADC Mode Control register (Figure 2a). When
        CHAN_ID is set to 1 keep the SCLK high for at least 25ns
        before the CS falling edge
        """
        print('conversionComplete')
        #self.spiRead()
        self.spiWriteReadBin(self.modeControlBit)
        
        return
    
    def readData(self, channelSelect = 0, scanControl = ''):
        """
        3-Wire Serial Interface, pg 15
        The serial data
        input, DIN, carries data into the control registers clocked
        in by the rising edge of SCLK. The serial data output,
        DOUT, delivers the conversion results and is clocked out
        by the falling edge of SCLK. DOUT is a 16-bit data word
        containing a 4-bit channel address, followed by a 12-bit
        conversion result led by the MSB when CHAN_ID is set
        to 1 in the ADC Mode Control register

        """
        if scanControl == '': scanControl = self.scanControl
  
        # send the mode control that will apply to the NEXT conversion
        # read back data of CURRENT conversion
        self.modeControl(scanControl, channelSelect)
        self.spiRead()
        
        if scanControl == 'MANUAL':
            ch, chVal = self.doutToChannelValue(channelSelect)
            print('ch =', ch, '  chVal =', chVal)
            
        elif scanControl in ['STANDARD_INT']:
            for i in range(channelSelect+1):
                print(self.doutToChannelValue(i))
        
        return
    
    def doutToChannelValue(self, ch):
        if self.debug: print('doutToChannelValue')
        doutBuf = self.readBuf
        if self.scanControl == 'MANUAL': ch = 0
        chWord = doutBuf[ch*2] << 8 | doutBuf[ch*2 + 1] # combine 2 bytes to one 16-bit word
        if self.debug: print('chWord =', chWord)
        chAddr = (chWord >> 12) & 0b1111 # first 4 bits
        chVal = (chWord) & 0b111111111111 # last 12 bits
        
        return (chAddr, chVal)
    
    def spiWriteReadBin(self, data):
        struct.pack_into('>H', self.writeBuf, 0, data)
        
        if self.debug: print('bin(din) =', bin(data))
        if self.debug: print('self.writeBuf', list(self.writeBuf))
        
        self.csPin.value(1)
        self.csPin.value(0)
        
        self.spi.write_readinto(self.writeBuf, self.readBuf)
        
        self.csPin.value(1)
        
        if self.debug: print('self.readBuf after', list(self.readBuf))
        
        return self.readBuf
    
    def spiRead(self):
        self.csPin.value(1)
        time.sleep_us(1)
        self.csPin.value(0)
        self.spi.readinto(self.readBuf)
        self.csPin.value(1)
        
        if self.debug: print('self.readBuf after', list(self.readBuf))
        
        return self.readBuf
    
    
    def deinit(self):
        self.spi.deinit()
        return



if __name__ == '__main__':
    pins = {
        'EOC': 4,
        'CS_ADC': 5,
        'CLK': 18,
        'MOSI': 23,
        'MISO': 19,
        'CNVST': 22
    }
    channels = [
        'AIN0',
        'AIN1',
        #'AIN2'
    ]

    
    adc = ADC_MAX11335(pins = pins, channels = channels,
                       reference = 'SINGLE_ENDED',
                       scanControl = 'STANDARD_INT',
                       debug = True)
  
    
    #adc.deinit()