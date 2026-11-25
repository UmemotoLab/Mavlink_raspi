#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_aruco_compat.py — ArUco 検出 API の OpenCV 4.6 / 4.7+ 互換層の試験。

対象（AR-hook1.py）:
  - build_aruco_detector()         : 4.7+ の ArucoDetector と 4.6 の
                                     aruco.detectMarkers() を切り替え、
                                     常に (corners, ids, rejected) を返す
  - aruco_create_detector_params() : DetectorParameters / DetectorParameters_create
                                     の選択ロジック
  - _aruco_normalize_detection()   : 戻り値の 3 要素正規化
  - aruco_ids_to_list()            : (N,1) / (N,) / None のすべてで例外なし

背景: Raspberry Pi 5 / Debian 12 の `sudo apt install python3-opencv` は OpenCV 4.6 で、
  `cv2.aruco.ArucoDetector` / `cv2.aruco.DetectorParameters()` が存在しない（実測:
  hasattr(cv2.aruco, "ArucoDetector") = False）。4.7+ の API を直接呼ぶと
  カメラ復旧後の追跡ループで AttributeError になるため、互換層を入れている。

実機・カメラ・OpenCV の有無に依存しません（fake の aruco モジュールだけを使う）。
実行:
    cd AR-hook && python3 -m pytest -q test_aruco_compat.py
"""
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# AR-hook1.py を import するためのモック（pymavlink / cv2 / pytz）を再利用する。
from test_failsafe_injection import _new_module  # noqa: E402


@pytest.fixture(scope="module")
def arhook():
    """AR-hook1.py をモジュールとして読み込む（main() は実行しない）。"""
    return _new_module()


# ───── fake の aruco モジュール（4.7+ / 4.6 相当） ─────
class _FakeParams:
    """cv2.aruco.DetectorParameters の最小互換。"""

    def __init__(self, created_by):
        self.created_by = created_by
        self.cornerRefinementMethod = None
        self.cornerRefinementWinSize = None
        self.cornerRefinementMaxIterations = None
        self.cornerRefinementMinAccuracy = None


class _FakeArucoDetector:
    """cv2.aruco.ArucoDetector の最小互換（4.7+）。"""

    def __init__(self, module, dictionary, params):
        self.module = module
        self.dictionary = dictionary
        self.params = params
        self.images = []

    def detectMarkers(self, image):
        self.images.append(image)
        return self.module.detect_result


class FakeAruco47:
    """OpenCV 4.7+ 相当（ArucoDetector / DetectorParameters あり）。"""

    DICT_4X4_50 = 10
    CORNER_REFINE_SUBPIX = 1

    def __init__(self, detect_result=None):
        self.detect_result = ([], None, []) if detect_result is None else detect_result
        self.dictionary_ids = []
        self.params_created = []
        self.detectors = []

    def getPredefinedDictionary(self, dictionary_id):
        self.dictionary_ids.append(dictionary_id)
        return ("dictionary", dictionary_id)

    def DetectorParameters(self):
        params = _FakeParams("DetectorParameters")
        self.params_created.append(params)
        return params

    def ArucoDetector(self, dictionary, params):
        detector = _FakeArucoDetector(self, dictionary, params)
        self.detectors.append(detector)
        return detector


class FakeAruco46:
    """OpenCV 4.6 相当（ArucoDetector 無し / DetectorParameters_create あり）。"""

    DICT_4X4_50 = 10
    CORNER_REFINE_SUBPIX = 1

    def __init__(self, detect_result=None):
        self.detect_result = ([], None, []) if detect_result is None else detect_result
        self.dictionary_ids = []
        self.params_created = []
        self.detect_calls = []

    def getPredefinedDictionary(self, dictionary_id):
        self.dictionary_ids.append(dictionary_id)
        return ("dictionary", dictionary_id)

    def DetectorParameters_create(self):
        params = _FakeParams("DetectorParameters_create")
        self.params_created.append(params)
        return params

    def detectMarkers(self, image, dictionary, parameters=None):
        self.detect_calls.append(
            {
                "image": image,
                "dictionary": dictionary,
                "parameters": parameters,
            }
        )
        return self.detect_result


# ───── 4.7+ / 4.6 の両方で (corners, ids, rejected) が返ること ─────
def test_opencv_47_uses_aruco_detector(arhook):
    """ArucoDetector がある環境では ArucoDetector 経由で検出する。"""
    fake = FakeAruco47(detect_result=([["corner"]], [[3]], []))
    detect, params, dictionary, label = arhook.build_aruco_detector(fake)

    assert label == arhook.ARUCO_API_DETECTOR
    assert fake.dictionary_ids == [FakeAruco47.DICT_4X4_50]
    assert dictionary == ("dictionary", FakeAruco47.DICT_4X4_50)
    assert len(fake.detectors) == 1
    assert fake.detectors[0].params is params
    # 4.6 側のモジュール関数は生えていない（選択ロジックの前提）
    assert not hasattr(fake, "detectMarkers")

    # corner refinement の設定値は従来どおり（既定値を変えていない）
    assert params.created_by == "DetectorParameters"
    assert params.cornerRefinementMethod == FakeAruco47.CORNER_REFINE_SUBPIX
    assert params.cornerRefinementWinSize == 5
    assert params.cornerRefinementMaxIterations == 30
    assert params.cornerRefinementMinAccuracy == 0.1

    corners, ids, rejected = detect("frame")
    assert fake.detectors[0].images == ["frame"]
    assert corners == [["corner"]]
    assert ids == [[3]]
    assert rejected == []
    assert arhook.aruco_ids_to_list(ids) == [3]


def test_opencv_46_uses_detect_markers(arhook):
    """ArucoDetector が無い環境では aruco.detectMarkers へ自動フォールバックする。"""
    fake = FakeAruco46(detect_result=([["corner"]], [[4]], ["rejected"]))
    detect, params, dictionary, label = arhook.build_aruco_detector(fake)

    assert label == arhook.ARUCO_API_LEGACY
    assert not hasattr(fake, "ArucoDetector")
    assert fake.dictionary_ids == [FakeAruco46.DICT_4X4_50]
    assert dictionary == ("dictionary", FakeAruco46.DICT_4X4_50)
    assert fake.params_created[0].created_by == "DetectorParameters_create"
    assert params.cornerRefinementMethod == FakeAruco46.CORNER_REFINE_SUBPIX
    assert params.cornerRefinementWinSize == 5
    assert params.cornerRefinementMaxIterations == 30
    assert params.cornerRefinementMinAccuracy == 0.1

    corners, ids, rejected = detect("frame")
    assert len(fake.detect_calls) == 1
    call = fake.detect_calls[0]
    assert call["image"] == "frame"
    assert call["dictionary"] == ("dictionary", FakeAruco46.DICT_4X4_50)
    assert call["parameters"] is params
    assert corners == [["corner"]]
    assert ids == [[4]]
    assert rejected == ["rejected"]
    assert arhook.aruco_ids_to_list(ids) == [4]


# ───── DetectorParameters / DetectorParameters_create の選択ロジック ─────
def test_params_prefers_detector_parameters(arhook):
    params, api = arhook.aruco_create_detector_params(FakeAruco47())
    assert api == "DetectorParameters"
    assert params.created_by == "DetectorParameters"


def test_params_falls_back_to_detector_parameters_create(arhook):
    params, api = arhook.aruco_create_detector_params(FakeAruco46())
    assert api == "DetectorParameters_create"
    assert params.created_by == "DetectorParameters_create"


def test_params_prefers_new_api_when_both_exist(arhook):
    fake = FakeAruco47()
    fake.DetectorParameters_create = lambda: _FakeParams("DetectorParameters_create")
    params, api = arhook.aruco_create_detector_params(fake)
    assert api == "DetectorParameters"
    assert params.created_by == "DetectorParameters"


def test_params_missing_both_raises_attribute_error(arhook):
    class _NoParamsApi:
        pass

    with pytest.raises(AttributeError):
        arhook.aruco_create_detector_params(_NoParamsApi())


def test_build_without_detection_api_raises_attribute_error(arhook):
    class _NoDetectApi:
        DICT_4X4_50 = 10
        CORNER_REFINE_SUBPIX = 1

        def getPredefinedDictionary(self, dictionary_id):
            return ("dictionary", dictionary_id)

        def DetectorParameters(self):
            return _FakeParams("DetectorParameters")

    with pytest.raises(AttributeError):
        arhook.build_aruco_detector(_NoDetectApi())


# ───── 戻り値の 3 要素正規化 ─────
def test_normalize_detection_returns_three_items(arhook):
    assert arhook._aruco_normalize_detection(([1], [2], [3])) == ([1], [2], [3])
    # 実装差で 2 要素（corners, ids）でも rejected を補う
    assert arhook._aruco_normalize_detection(([1], [2])) == ([1], [2], [])
    assert arhook._aruco_normalize_detection(None) == ([], None, [])
    with pytest.raises(TypeError):
        arhook._aruco_normalize_detection(42)


# ───── ids の形状（(N,1) / (N,) / None / 0 件）で例外が出ないこと ─────
@pytest.mark.parametrize("fake_class", [FakeAruco47, FakeAruco46])
@pytest.mark.parametrize(
    "ids, expected",
    [
        ([[1], [2]], [1, 2]),  # OpenCV 4.x の (N,1)
        ([1, 2], [1, 2]),      # OpenCV 5.x の (N,)
        (None, []),            # 未検出
        ([], []),              # 0 件検出
    ],
)
def test_ids_shapes_do_not_raise(arhook, fake_class, ids, expected):
    fake = fake_class(detect_result=([], ids, []))
    detect, _params, _dictionary, _label = arhook.build_aruco_detector(fake)

    corners, detected_ids, rejected = detect("frame")
    assert detected_ids is ids
    assert corners == []
    assert rejected == []
    assert arhook.aruco_ids_to_list(detected_ids) == expected


@pytest.mark.parametrize("fake_class", [FakeAruco47, FakeAruco46])
def test_ids_numpy_shapes_do_not_raise(arhook, fake_class):
    """numpy がある環境では実物の ndarray 形状でも確認する。"""
    np = pytest.importorskip("numpy")
    cases = [
        (np.array([[5]], dtype=np.int32), [5]),
        (np.array([5], dtype=np.int32), [5]),
        (None, []),
        (np.empty((0, 1), dtype=np.int32), []),
    ]
    for ids, expected in cases:
        fake = fake_class(detect_result=([], ids, []))
        detect, _params, _dictionary, _label = arhook.build_aruco_detector(fake)
        _corners, detected_ids, _rejected = detect("frame")
        assert arhook.aruco_ids_to_list(detected_ids) == expected


# ───── 追跡ループが互換層と起動ログを使っていること（静的な配線確認） ─────
def test_camera_tracker_loop_uses_compat_layer():
    src = AR_HOOK.read_text(encoding="utf-8")
    start = src.index("def camera_tracker_loop(")
    end = src.index("def control_loop(", start)
    body = src[start:end]

    assert "build_aruco_detector()" in body
    assert "detect_markers(img)" in body
    assert 'print(f"ArUco: {aruco_api_label}")' in body
    # 4.7+ 専用 API の直接利用が残っていないこと（4.6 実機で AttributeError になる）
    assert "aruco.ArucoDetector(" not in body
    assert "aruco.DetectorParameters(" not in body


def test_api_labels_are_stable(arhook):
    """起動ログの 1 行表記（実機での版確認用）。"""
    assert arhook.ARUCO_API_DETECTOR == "ArucoDetector (OpenCV 4.7+)"
    assert arhook.ARUCO_API_LEGACY == "detectMarkers (OpenCV 4.6 互換)"
