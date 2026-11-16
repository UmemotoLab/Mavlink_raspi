"""
SD card driver for standard Python (Raspberry Pi 5 対応版)
"""

def const(x):
    return x

import time

_CMD_TIMEOUT = const(100)
_R1_IDLE_STATE = const(1 << 0)
_R1_ILLEGAL_COMMAND = const(1 << 2)
_TOKEN_CMD25 = const(0xfc)
_TOKEN_STOP_TRAN = const(0xfd)
_TOKEN_DATA = const(0xfe)

class SDCard:
    def __init__(self, spi, cs, baudrate=1000000):
        self.spi = spi
        self.cs = cs
        
        self.cmdbuf = bytearray(6)
        self.dummybuf = bytearray(512)
        self.tokenbuf = bytearray(1)
        for i in range(512):
            self.dummybuf[i] = 0xff
        
        self.init_card(baudrate)

    def init_card(self, baudrate):
        # init CS pin
        self.cs.init(self.cs.OUT, value=1)
        self.cs(1)
        self.spi.write(bytearray(10))
        
        # init the card in SPI mode
        self.spi.init(baudrate=100000)  # 100kHz for initialization
        
        for _ in range(16):
            self.spi.write(b'\xff')
        
        # CMD0: init card; should return _R1_IDLE_STATE (allow 5 attempts)
        for _ in range(5):
            if self.cmd(0, 0, 0x95) == _R1_IDLE_STATE:
                break
        else:
            raise OSError("no SD card")
        
        # CMD8: determine card version
        r = self.cmd(8, 0x01AA, 0x87, 4)
        if r == _R1_IDLE_STATE:
            self.init_card_v2()
        elif r == (_R1_IDLE_STATE | _R1_ILLEGAL_COMMAND):
            self.init_card_v1()
        else:
            raise OSError("couldn't determine SD card version")
        
        # get the number of sectors
        # CMD9: response R2 (R1 byte + 16-byte block read)
        if self.cmd(9, 0, 0, 0, False) != 0:
            raise OSError("no response from SD card")
        csd = bytearray(16)
        self.readinto(csd)
        if csd[0] & 0xc0 == 0x40:
            # CSD version 2.0
            self.sectors = ((csd[8] << 8 | csd[9]) + 1) * 1024
        elif csd[0] & 0xc0 == 0x00:
            # CSD version 1.0 (old, <=2GB)
            c_size = (csd[6] & 0x3) << 10 | csd[7] << 2 | csd[8] >> 6
            c_size_mult = (csd[9] & 0x3) << 1 | csd[10] >> 7
            read_bl_len = csd[5] & 0xf
            self.sectors = (c_size + 1) * (2 ** (c_size_mult + 2)) * (2 ** (read_bl_len - 9))
        else:
            raise OSError("SD card CSD format not supported")
        # CMD16: set block length to 512 bytes
        if self.cmd(16, 512, 0) != 0:
            raise OSError("can't set 512 block size")
        
        # set to high data rate now that it's initialised
        self.spi.init(baudrate=baudrate)

    def init_card_v1(self):
        for _ in range(_CMD_TIMEOUT):
            self.cmd(55, 0, 0)
            if self.cmd(41, 0, 0) == 0:
                self.cdv = 512
                return
            time.sleep(0.050)  # sleep_ms(50) の代替
        raise OSError("timeout waiting for v1 card")

    def init_card_v2(self):
        for _ in range(_CMD_TIMEOUT):
            time.sleep(0.050)  # sleep_ms(50) の代替
            self.cmd(58, 0, 0, 4)
            self.cmd(55, 0, 0)
            if self.cmd(41, 0x40000000, 0) == 0:
                self.cmd(58, 0, 0, 4)
                self.cdv = 1
                return
        raise OSError("timeout waiting for v2 card")

    def cmd(self, cmd, arg, crc, final=0, release=True, skip1=False):
        self.cs(0)
        
        # create and send the command
        buf = self.cmdbuf
        buf[0] = 0x40 | cmd
        # 32ビット引数を4バイトに正しく分解
        buf[1] = (arg >> 24) & 0xFF
        buf[2] = (arg >> 16) & 0xFF
        buf[3] = (arg >> 8) & 0xFF
        buf[4] = arg & 0xFF
        buf[5] = crc
        self.spi.write(buf)
        
        if skip1:
            self.spi.read(1)
        
        # wait for the response (response[7] == 0)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xff)
            response = self.tokenbuf[0]
            if not (response & 0x80):
                # CMD17 and CMD24 return additional data
                for j in range(final):
                    self.spi.read(1)
                if release:
                    self.cs(1)
                    self.spi.write(b'\xff')
                return response
        
        # timeout
        self.cs(1)
        self.spi.write(b'\xff')
        return -1

    def readinto(self, buf):
        self.cs(0)
        
        # read until start byte (0xff)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xff)
            if self.tokenbuf[0] == _TOKEN_DATA:
                break
            time.sleep(0.001)  # sleep_ms(1) の代替
        else:
            self.cs(1)
            raise OSError("timeout waiting for response")
        
        # read data
        self.spi.readinto(buf)
        
        # read checksum
        self.spi.read(2)
        
        self.cs(1)
        self.spi.write(b'\xff')

    def write(self, token, buf):
        self.cs(0)
        
        # send: start of block, data, checksum
        self.spi.write([token])
        self.spi.write(buf)
        self.spi.write(b'\xff')
        self.spi.write(b'\xff')
        
        # check the response
        self.spi.readinto(self.tokenbuf, 0xff)
        if (self.tokenbuf[0] & 0x1f) != 0x05:
            self.cs(1)
            raise OSError("rejected data")
        
        # wait for write to finish
        while True:
            self.spi.readinto(self.tokenbuf, 0xff)
            if self.tokenbuf[0]:
                break
        
        self.cs(1)
        self.spi.write(b'\xff')

    def write_token(self, token):
        self.cs(0)
        self.spi.write([token])
        self.spi.write(b'\xff')
        # wait for write to finish
        while True:
            self.spi.readinto(self.tokenbuf, 0xff)
            if self.tokenbuf[0]:
                break
        self.cs(1)
        self.spi.write(b'\xff')

    def readblocks(self, block_num, buf):
        nblocks = len(buf) // 512
        assert nblocks and not len(buf) % 512, "Buffer length is invalid"
        if nblocks == 1:
            # CMD17: set read address for single block
            if self.cmd(17, block_num * self.cdv, 0, release=False) != 0:
                raise OSError(5)  # EIO
            # receive the data and release card
            self.readinto(buf)
        else:
            # CMD18: set read address for multiple blocks
            if self.cmd(18, block_num * self.cdv, 0, release=False) != 0:
                raise OSError(5)  # EIO
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                # receive the data and release card
                self.readinto(mv[offset:offset + 512])
                offset += 512
                nblocks -= 1
            if self.cmd(12, 0, 0, skip1=True):
                raise OSError(5)  # EIO

    def writeblocks(self, block_num, buf):
        nblocks, err = divmod(len(buf), 512)
        assert nblocks and not err, "Buffer length is invalid"
        if nblocks == 1:
            # CMD24: set write address for single block
            if self.cmd(24, block_num * self.cdv, 0) != 0:
                raise OSError(5)  # EIO
            
            # send the data
            self.write(_TOKEN_DATA, buf)
        else:
            # CMD25: set write address for first block
            if self.cmd(25, block_num * self.cdv, 0) != 0:
                raise OSError(5)  # EIO
            # send the data
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                self.write(_TOKEN_CMD25, mv[offset:offset + 512])
                offset += 512
                nblocks -= 1
            self.write_token(_TOKEN_STOP_TRAN)
