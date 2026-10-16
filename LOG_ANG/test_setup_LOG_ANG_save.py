"""LOG_ANG 保存フロー focused test（実機無接続・副作用なし）。

確認点:
- 明示の保存コマンド送信がないこと
- 保存待機・再試行・最終読み戻しが残っていること
- EEPROM保存成功のような誤解を招く表示がなく、電源再投入後の
  永続化を保証しない旨が明示されていること
- --check が PASS すること
"""
import subprocess
import sys
from pathlib import Path

TARGET = Path(__file__).with_name("setup_LOG_ANG.py")


def _text():
    return TARGET.read_text(encoding="utf-8")


def test_no_explicit_storage_command():
    text = _text()
    assert "PREFLIGHT_STORAGE" not in text, "明示の保存コマンド識別子が残っている"
    assert "EEPROMへの保存成功" not in text
    assert "EEPROMに保存されました" not in text
    assert "再度EEPROMに保存中" not in text
    assert "設定をEEPROMに保存中" not in text


def test_autosave_wait_retry_and_readback():
    text = _text()
    # 保存待機
    assert "自動保存完了を待機" in text, "自動保存の待機表示がない"
    # 再試行フロー（初期設定の再試行 + 不一致分の再設定）
    assert "def set_parameter_reliable" in text
    assert "max_retries" in text
    assert "failed_params" in text, "不一致分の再設定フローがない"
    assert "再設定分の自動保存完了を待機" in text
    # 最終読み戻し
    assert "def verify_parameter" in text
    assert "最終確認を実施中" in text
    assert "param_request_read_send" in text, "読み戻し要求がない"


def test_no_misleading_result_message():
    text = _text()
    # 成功時は読み戻し一致の表現にとどめ、電源再投入後の保証をしない
    assert "読み戻しで一致を確認" in text
    assert "永続化を保証するものではない" in text
    assert "電源再投入" in text
    # RAM/EEPROM の成否断定が残っていないこと
    assert "RAM内に設定されています" not in text


def test_check_mode_passes():
    proc = subprocess.run(
        [sys.executable, str(TARGET), "--check"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert proc.returncode == 0, f"--check が失敗: {proc.stdout}\n{proc.stderr}"
    assert "RESULT: PASS" in proc.stdout


if __name__ == "__main__":
    test_no_explicit_storage_command()
    test_autosave_wait_retry_and_readback()
    test_no_misleading_result_message()
    test_check_mode_passes()
    print("ALL FOCUSED TESTS PASSED")
