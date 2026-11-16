#!/usr/bin/env python3
# simple_init_test.py - 初期化テスト

from archive.microSD.sdcard_adapter import Pin, SPI, const

try:
    print("=== 初期化テスト ===")
    
    # Pin初期化テスト
    print("1. Pin初期化テスト")
    cs = Pin(8, Pin.OUT)
    
    # Pin.init() with value テスト
    print("2. Pin.init(value=1) テスト")
    cs.init(Pin.OUT, value=1)
    
    # SPI初期化テスト
    print("3. SPI初期化テスト")
    spi = SPI(0, 0)
    
    print("✓ 全ての初期化テストが成功しました")
    
    # SDCard初期化テスト
    print("4. SDCard初期化テスト")
    from archive.microSD.sdcard import SDCard
    sd = SDCard(spi, cs)
    print(f"✓ SDCard初期化成功（セクタ数: {sd.sectors}）")
    
    spi.close()
    
except Exception as e:
    print(f"エラー: {e}")
    import traceback
    traceback.print_exc()
