#!/bin/bash
# 2026-11-21: retrain with the 11-20 lab runs and Livox reflectivity features.
# Compares leave-one-run-out AP with and without --refl, then saves both models.
set -e
cd "$(dirname "$0")/../.."
D=artifacts/lidar-person-classifier-2026-11-01
RUNS=$(ls -d artifacts/person-lidar-2026-1{0-30,1-03,1-07,1-08,1-09,1-20}/*/ | grep -vE '/(trial|trial-lab|trial-lab2|apt-desk2-trial|apt-smoke-r1|bias-check-r1)/$')
echo "runs: $(echo $RUNS | wc -w)"
python3 src/tools/perception/lidar_person_dataset.py $RUNS -o $D/own_1121.npz \
    --overrides $D/label_overrides.json --jobs 16
python3 src/tools/perception/lidar_person_classifier.py --own $D/own_1121.npz --save $D/model_base_1121.joblib > $D/results_base_1121.txt
python3 src/tools/perception/lidar_person_classifier.py --own $D/own_1121.npz --refl --save $D/model_refl_1121.joblib > $D/results_refl_1121.txt
date > $D/retrain_1121.done
