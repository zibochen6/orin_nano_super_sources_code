#!/usr/bin/env bash

ENV_DIR="/opt/seeed/development_guide/envs"

echo "=============================="
echo " Testing base environment"
echo "=============================="
python3 "$ENV_DIR/check_base_pkgs.py" || echo "[FAIL] base env failed"

declare -A ENVS
ENVS[".tf"]="tensorflow keras numpy"
ENVS[".gpio"]="Jetson.GPIO"
ENVS[".mediapipe"]="mediapipe cv2 numpy"
ENVS[".opencv"]="cv2 numpy"
ENVS[".qr"]="qrcode pyzbar PIL"
ENVS[".yolo"]="torch torchvision ultralytics cv2"

for env in "${!ENVS[@]}"; do
    echo
    echo "=============================="
    echo " Testing env: $env"
    echo "=============================="

    source "$ENV_DIR/$env/bin/activate"

    python "$ENV_DIR/test_env.py" "$env" ${ENVS[$env]}
    RESULT=$?

    deactivate

    if [ $RESULT -ne 0 ]; then
        echo "[FAIL] $env failed"
    else
        echo "[OK] $env passed"
    fi
done

