#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Check availability of main AI/LLM/acceleration/multimedia packages.
- No network calls.
- No model downloads.
- Skip: local workspace pkgs, data/science stack, notebook/dev, hardware/Grove stack.
"""

import importlib
import os
import platform
import sys
import traceback
from datetime import datetime


def try_import(mod_name: str):
    """Return (ok, module_or_none, err_str_or_none)."""
    try:
        m = importlib.import_module(mod_name)
        return True, m, None
    except Exception as e:
        return False, None, f"{type(e).__name__}: {e}"


def get_version(module, mod_name: str):
    """Best-effort version extraction."""
    for attr in ("__version__", "VERSION", "version"):
        if hasattr(module, attr):
            v = getattr(module, attr)
            try:
                return str(v)
            except Exception:
                pass
    # Some libs expose version via submodule
    if mod_name == "tensorrt":
        # tensorrt.__version__ exists on many installs; if not, return unknown
        return getattr(module, "__version__", "unknown")
    return "unknown"


def header():
    print("=" * 80)
    print("Main Package Availability Check")
    print(f"Time: {datetime.now().isoformat(timespec='seconds')}")
    print(f"Python: {sys.version.split()[0]}  |  Platform: {platform.platform()}")
    print(f"Executable: {sys.executable}")
    print(f"PWD: {os.getcwd()}")
    print("=" * 80)


def ok_line(name, detail=""):
    print(f"[  OK  ] {name}{(' | ' + detail) if detail else ''}")


def fail_line(name, detail=""):
    print(f"[ FAIL ] {name}{(' | ' + detail) if detail else ''}")


def warn_line(name, detail=""):
    print(f"[ WARN ] {name}{(' | ' + detail) if detail else ''}")


def test_torch():
    ok, torch, err = try_import("torch")
    if not ok:
        fail_line("torch", err)
        return False

    ver = getattr(torch, "__version__", "unknown")
    # minimal tensor op
    try:
        x = torch.tensor([1.0, 2.0, 3.0])
        y = x * 2
        _ = float(y.sum().item())
        cuda = torch.cuda.is_available()
        ok_line("torch", f"version={ver}, cuda_available={cuda}")
        # If CUDA available, do a tiny cuda op
        if cuda:
            z = torch.tensor([1.0], device="cuda")
            _ = float((z + 1).item())
            ok_line("torch.cuda", f"device={torch.cuda.get_device_name(0)}")
        else:
            warn_line("torch.cuda", "CUDA not available (this may be expected)")
        return True
    except Exception as e:
        fail_line("torch(minimal-op)", f"{type(e).__name__}: {e}")
        return False


def test_jax():
    ok, jax, err = try_import("jax")
    if not ok:
        fail_line("jax", err)
        return False
    ok2, jnp, err2 = try_import("jax.numpy")
    if not ok2:
        fail_line("jax.numpy", err2)
        return False

    try:
        a = jnp.array([1.0, 2.0, 3.0])
        b = a * 3
        _ = float(b.sum())
        devs = jax.devices()
        ok_line("jax", f"version={getattr(jax, '__version__', 'unknown')}, devices={len(devs)}")
        return True
    except Exception as e:
        fail_line("jax(minimal-op)", f"{type(e).__name__}: {e}")
        return False


def test_onnx_ort():
    ok, onnx, err = try_import("onnx")
    if ok:
        ok_line("onnx", f"version={getattr(onnx, '__version__', 'unknown')}")
    else:
        fail_line("onnx", err)

    ok2, ort, err2 = try_import("onnxruntime")
    if not ok2:
        fail_line("onnxruntime", err2)
        return False

    try:
        providers = ort.get_available_providers()
        ok_line("onnxruntime", f"version={getattr(ort, '__version__', 'unknown')}, providers={providers}")
        # Don't create sessions or load models; provider list is enough for availability.
        return ok and True
    except Exception as e:
        fail_line("onnxruntime(providers)", f"{type(e).__name__}: {e}")
        return False


def test_tensorrt():
    ok, trt, err = try_import("tensorrt")
    if not ok:
        # On some systems TensorRT python binding may be absent even if TRT exists
        warn_line("tensorrt", f"not importable: {err}")
        return False

    try:
        ver = getattr(trt, "__version__", "unknown")
        # minimal: create logger
        logger = trt.Logger(trt.Logger.WARNING)
        ok_line("tensorrt", f"version={ver}, logger_ok=True")
        return True
    except Exception as e:
        fail_line("tensorrt(minimal)", f"{type(e).__name__}: {e}")
        return False


def test_basic_imports(mods):
    all_ok = True
    for name in mods:
        ok, m, err = try_import(name)
        if ok:
            ok_line(name, f"version={get_version(m, name)}")
        else:
            fail_line(name, err)
            all_ok = False
    return all_ok


def test_mediapipe():
    ok, mp, err = try_import("mediapipe")
    if not ok:
        fail_line("mediapipe", err)
        return False
    try:
        # minimal: access solutions namespace
        _ = mp.solutions
        ok_line("mediapipe", f"version={getattr(mp, '__version__', 'unknown')}")
        return True
    except Exception as e:
        fail_line("mediapipe(minimal)", f"{type(e).__name__}: {e}")
        return False


def test_opencv():
    ok, cv2, err = try_import("cv2")
    if not ok:
        fail_line("opencv(cv2)", err)
        return False
    try:
        # minimal: create empty image and do a conversion
        import numpy as _np  # only used internally for minimal op; ok to depend here
        img = _np.zeros((10, 10, 3), dtype=_np.uint8)
        _ = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok_line("opencv-python(cv2)", f"version={getattr(cv2, '__version__', 'unknown')}")
        return True
    except Exception as e:
        fail_line("opencv(minimal)", f"{type(e).__name__}: {e}")
        return False


def test_ultralytics():
    ok, ul, err = try_import("ultralytics")
    if not ok:
        fail_line("ultralytics", err)
        return False
    try:
        # Don't instantiate YOLO() (may trigger downloads). Just import and check version.
        ver = getattr(ul, "__version__", "unknown")
        ok_line("ultralytics", f"version={ver}")
        return True
    except Exception as e:
        fail_line("ultralytics(minimal)", f"{type(e).__name__}: {e}")
        return False


def test_dlib():
    ok, dlib, err = try_import("dlib")
    if not ok:
        fail_line("dlib", err)
        return False
    try:
        # minimal: check if dlib has cuda support flag (if present)
        cuda = getattr(dlib, "DLIB_USE_CUDA", None)
        ok_line("dlib", f"version={getattr(dlib, '__version__', 'unknown')}, DLIB_USE_CUDA={cuda}")
        return True
    except Exception as e:
        fail_line("dlib(minimal)", f"{type(e).__name__}: {e}")
        return False


def main():
    header()

    results = {}

    # Core DL frameworks
    results["torch"] = test_torch()
    results["jax"] = test_jax()

    print("-" * 80)

    # Inference / acceleration
    results["onnx+onnxruntime"] = test_onnx_ort()
    results["tensorrt"] = test_tensorrt()

    print("-" * 80)

    # LLM ecosystem libs (no network calls)
    results["llm-ecosystem"] = test_basic_imports([
        "ollama",
        "openai",
        "modelscope",
        "dashscope",
    ])

    print("-" * 80)

    # Vision / multimodal
    results["opencv"] = test_opencv()
    results["mediapipe"] = test_mediapipe()
    results["ultralytics"] = test_ultralytics()
    results["dlib"] = test_dlib()

    print("-" * 80)

    # Summary
    total = len(results)
    passed = sum(1 for v in results.values() if v)
    failed = total - passed

    print("SUMMARY")
    print(f"  Total checks: {total}")
    print(f"  Passed     : {passed}")
    print(f"  Failed/Warn: {failed}")
    print("=" * 80)

    # Exit code: 0 if all pass, 1 otherwise
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()

