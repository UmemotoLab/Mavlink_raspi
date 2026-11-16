#!/usr/bin/env python3
# spi_auto_cs_test.py - CS自動制御版

import spidev
import time

def test_auto_cs():
    """spidevの自動CS制御を使用したテスト"""
    
    try:
        # SPI初期化（CS制御はspidevが自動で行う）
        spi = spidev.SpiDev()
        spi.open(0, 0)  # /dev/spidev0.0 使用、CS0（GPIO8）自動制御
        spi.max_speed_hz = 400000  # 400kHz
        spi.mode = 0
        
        print("=== SPI自動CS制御テスト ===")
        print(f"使用SPI: /dev/spidev0.0")
        print(f"CS制御: 自動（spidev）- GPIO8")
        print(f"速度: {spi.max_speed_hz}Hz")
        
        # 1. 初期化クロック送信（CS制御は自動）
        print("\n1. 初期化クロック送信")
        dummy_clocks = [0xFF] * 10
        response = spi.xfer2(dummy_clocks)
        print(f"   応答: {[hex(x) for x in response[:5]]}...") # 最初の5バイトのみ表示
        
        time.sleep(0.01)
        
        # 2. CMD0送信（CS制御は自動）
        print("2. CMD0送信")
        cmd0 = [0x40, 0x00, 0x00, 0x00, 0x00, 0x95]
        print(f"   送信: {[hex(x) for x in cmd0]}")
        
        # CMD0 + R1応答待ちを一度に送信
        cmd_and_response = cmd0 + [0xFF] * 8  # CMD0 + 8バイトの応答待ち
        result = spi.xfer2(cmd_and_response)
        
        print(f"   全応答: {[hex(x) for x in result]}")
        
        # 応答解析
        response_part = result[6:]  # CMD0の後の部分
        print(f"   R1応答部分: {[hex(x) for x in response_part]}")
        
        valid_response = False
        for i, byte_val in enumerate(response_part):
            if byte_val != 0xFF:
                print(f"   ✓ 有効応答 [{i}]: {hex(byte_val)}")
                if byte_val == 0x01:
                    print("   ✓ SDカード IDLE状態（正常）")
                    valid_response = True
                elif byte_val == 0x00:
                    print("   ✓ SDカード 正常状態")
                    valid_response = True
                elif (byte_val & 0x80) == 0:  # MSBが0なら有効な応答
                    print(f"   △ 応答コード: {hex(byte_val)} ({bin(byte_val)})")
                    valid_response = True
                break
        
        # 3. 詳細テスト: CMD8（SDカードバージョン確認）
        if valid_response:
            print("\n3. CMD8送信（SDカードバージョン確認）")
            cmd8 = [0x48, 0x00, 0x00, 0x01, 0xAA, 0x87]  # CMD8
            cmd8_response = cmd8 + [0xFF] * 8
            result8 = spi.xfer2(cmd8_response)
            print(f"   CMD8応答: {[hex(x) for x in result8]}")
        
        # 4. 結果判定
        print(f"\n=== テスト結果 ===")
        if valid_response:
            print("✓ SDカードとの通信が確認できました")
            print("  配線とSDカードは正常に動作しています")
            print("  次のステップ: SDカードライブラリのテスト")
            return True
        else:
            print("✗ SDカードからの有効な応答がありません")
            print("  確認事項：")
            print("  - microSDカードが挿入されているか")
            print("  - 配線が正しいか（特にMISO線）")
            print("  - SDカードが故障していないか")
            return False
        
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        try:
            spi.close()
        except:
            pass

if __name__ == "__main__":
    success = test_auto_cs()
    if success:
        print("\n次は完全なSDカードライブラリのテストに進めます")
    else:
        print("\n配線とハードウェアを再確認してください")
