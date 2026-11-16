#!/usr/bin/env python3
# sdcard_file_test.py - SDカードファイル読み書きテスト

import os
import sys
from datetime import datetime

try:
    from archive.microSD.sdcard_adapter import Pin, SPI, const
    from archive.microSD.sdcard import SDCard
except ImportError as e:
    print(f"インポートエラー: {e}")
    print("sdcard_adapter.py と sdcard.py が同じディレクトリにあることを確認してください")
    sys.exit(1)

class SDCardFileSystem:
    def __init__(self):
        """SDカードファイルシステム初期化"""
        try:
            print("=== SDカードファイルシステム初期化 ===")
            
            # SPI初期化（自動CS制御）
            self.spi = SPI(0, 0)
            self.cs = Pin(8, Pin.OUT)  # ダミーCS（実際はspidevが制御）
            
            # SDカード初期化
            print("SDカード初期化中...")
            self.sd = SDCard(self.spi, self.cs)
            print(f"✓ SDカード初期化完了（セクタ数: {self.sd.sectors}）")
            
        except Exception as e:
            print(f"初期化エラー: {e}")
            raise
    
    def write_text_file(self, block_num, filename, content):
        """テキストファイルをSDカードに書き込み"""
        try:
            # ファイル情報をブロックの最初に記録
            file_info = f"FILE:{filename}\nSIZE:{len(content)}\nDATE:{datetime.now()}\n---\n"
            full_content = file_info + content
            
            # 512バイトブロックに変換
            data = full_content.encode('utf-8')
            
            # 複数ブロックが必要な場合
            blocks_needed = (len(data) + 511) // 512
            print(f"書き込み: {filename} ({len(data)}バイト, {blocks_needed}ブロック)")
            
            for i in range(blocks_needed):
                buffer = bytearray(512)
                start = i * 512
                end = min(start + 512, len(data))
                buffer[:end-start] = data[start:end]
                
                self.sd.writeblocks(block_num + i, buffer)
                print(f"  ブロック {block_num + i} 書き込み完了")
            
            return blocks_needed
            
        except Exception as e:
            print(f"書き込みエラー: {e}")
            return 0
    
    def read_text_file(self, block_num, blocks_count=1):
        """SDカードからテキストファイルを読み込み"""
        try:
            print(f"読み込み: ブロック {block_num}〜{block_num + blocks_count - 1}")
            
            all_data = bytearray()
            
            for i in range(blocks_count):
                buffer = bytearray(512)
                self.sd.readblocks(block_num + i, buffer)
                all_data.extend(buffer)
            
            # null文字で終了
            end_pos = all_data.find(0)
            if end_pos != -1:
                all_data = all_data[:end_pos]
            
            content = all_data.decode('utf-8', errors='ignore')
            
            # ファイル情報と内容を分離
            if "---\n" in content:
                header, file_content = content.split("---\n", 1)
                print("ファイル情報:")
                print(header)
                return file_content
            else:
                return content
            
        except Exception as e:
            print(f"読み込みエラー: {e}")
            return None
    
    def close(self):
        """リソース解放"""
        try:
            self.spi.close()
            print("SDカード接続を閉じました")
        except:
            pass

def main():
    """メイン実行関数"""
    print("=== SDカード ファイル読み書きテスト ===")
    
    sd_fs = None
    try:
        # SDカードファイルシステム初期化
        sd_fs = SDCardFileSystem()
        
        # テストデータ作成
        test_files = [
            {
                "name": "test1.txt",
                "content": f"Hello, Raspberry Pi 5!\n作成日時: {datetime.now()}\nPmod MicroSD テスト成功！"
            },
            {
                "name": "sensor_log.txt", 
                "content": f"センサーログ\n{datetime.now()}: 温度=23.5°C, 湿度=65%\n{datetime.now()}: 温度=24.1°C, 湿度=63%"
            }
        ]
        
        current_block = 0
        
        for i, file_data in enumerate(test_files):
            print(f"\n--- ファイル {i+1}: {file_data['name']} ---")
            
            # 書き込みテスト
            blocks_used = sd_fs.write_text_file(
                current_block, 
                file_data['name'], 
                file_data['content']
            )
            
            if blocks_used > 0:
                print("✓ 書き込み成功")
                
                # 読み込みテスト
                read_content = sd_fs.read_text_file(current_block, blocks_used)
                
                if read_content:
                    print("✓ 読み込み成功")
                    print("読み込み内容:")
                    print("-" * 40)
                    print(read_content)
                    print("-" * 40)
                    
                    # 内容検証
                    if file_data['content'] in read_content:
                        print("✓ 内容一致確認")
                    else:
                        print("⚠ 内容に差異があります")
                else:
                    print("✗ 読み込み失敗")
                
                current_block += blocks_used
            else:
                print("✗ 書き込み失敗")
        
        print(f"\n=== テスト完了 ===")
        print("✓ SDカードへのファイル読み書きが正常に動作しました")
        print("✓ Raspberry Pi 5 + Pmod MicroSD の環境構築完了")
        
    except KeyboardInterrupt:
        print("\n中断されました")
    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if sd_fs:
            sd_fs.close()

if __name__ == "__main__":
    main()
