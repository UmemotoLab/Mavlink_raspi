# sdcard_adapter.py - value引数対応版
import spidev
import time

def const(x):
    return x

class Pin:
    """CS制御を spidev に委ねるダミーPinクラス"""
    OUT = 1
    IN = 0
    
    def __init__(self, pin_num, mode=None):
        self.pin = pin_num
        self.mode = mode
        # 実際のGPIO制御は行わない（spidevが自動制御）
        print(f"Pin {pin_num}: spidev自動制御モード")
    
    def init(self, mode=None, value=None):
        """MicroPython互換のinitメソッド（value引数対応）"""
        if mode is not None:
            self.mode = mode
        if value is not None:
            # valueが指定されても実際のGPIO制御は行わない
            pass
        print(f"Pin {self.pin}.init(mode={mode}, value={value}) - spidev自動制御のため無視")
    
    def __call__(self, value):
        """CS制御はspidevが自動で行うため、何もしない"""
        pass
    
    def value(self, val=None):
        """値の設定/取得（ダミー）"""
        return 1 if val is None else None
    
    def on(self):
        """便利メソッド（ダミー）"""
        pass
    
    def off(self):
        """便利メソッド（ダミー）"""
        pass

class SPI:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1000000  # 1MHz
        self.spi.mode = 0
        print(f"SPI初期化: /dev/spidev{bus}.{device}, 速度: {self.spi.max_speed_hz}Hz")
    
    def init(self, baudrate=1000000, polarity=0, phase=0):
        """MicroPython互換のinitメソッド"""
        self.spi.max_speed_hz = baudrate
        self.spi.mode = (polarity << 1) | phase
        print(f"SPI再設定: 速度={baudrate}Hz, モード={self.spi.mode}")
    
    def write(self, data):
        """データ書き込み"""
        if isinstance(data, int):
            # 単一の値の場合
            self.spi.xfer2([data])
        elif isinstance(data, (bytes, bytearray)):
            # バイト列の場合
            self.spi.xfer2(list(data))
        elif isinstance(data, list):
            # リストの場合
            self.spi.xfer2(data)
    
    def read(self, length, write_data=0xFF):
        """データ読み込み"""
        read_data = [write_data] * length
        result = self.spi.xfer2(read_data)
        return bytes(result)
    
    def readinto(self, buf, write_data=0xFF):
        """バッファに読み込み"""
        read_data = [write_data] * len(buf)
        result = self.spi.xfer2(read_data)
        for i in range(min(len(result), len(buf))):
            buf[i] = result[i]
    
    def close(self):
        """SPI接続を閉じる"""
        self.spi.close()
        print("SPI接続を閉じました")

# MicroPython互換関数
def sleep_ms(ms):
    """MicroPython time.sleep_ms の代替"""
    time.sleep(ms / 1000.0)

def sleep_us(us):
    """MicroPython time.sleep_us の代替"""
    time.sleep(us / 1000000.0)
