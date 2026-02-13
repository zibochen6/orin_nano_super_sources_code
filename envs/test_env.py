import sys
import importlib

def test_import(pkg):
    try:
        importlib.import_module(pkg)
        print(f"[OK] import {pkg}")
        return True
    except Exception as e:
        print(f"[FAIL] import {pkg}: {e}")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python test_env.py <env_name> [packages...]")
        sys.exit(1)

    env_name = sys.argv[1]
    packages = sys.argv[2:]

    print(f"Python executable: {sys.executable}")
    print(f"Python version   : {sys.version}")
    print(f"Testing env      : {env_name}")
    print("-" * 40)

    failed = False
    for pkg in packages:
        if not test_import(pkg):
            failed = True

    if failed:
        sys.exit(1)

if __name__ == "__main__":
    main()

