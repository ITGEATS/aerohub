#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
target="$root/deployment/dpv_slam/models/dpvo.pth"
artifact_root=${DPV_SLAM_ARTIFACT_ROOT:-$root/log/workspaces/dpv_slam}
wheelhouse="$artifact_root/wheelhouse"
cuda_packages="$artifact_root/cuda-packages"
model_cache="$artifact_root/models/models.zip"
model_archive_checksum=89f43de2c92676ddcf7f49e8ae3f8940b7af73f549a7a5b8308850b67de78479
model_checksum=30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12
model_url='https://www.dropbox.com/scl/fo/2g6e134ruh1acmaye5bmy/AK0DY9WLZ25elq-7N7sJcB0/models.zip?rlkey=4j336vcc4h324j4p4ji3ppl0t&dl=1'

download_checked() {
  local checksum=$1
  local url=$2
  local output=$3
  local partial="$output.partial"

  if echo "$checksum  $output" | sha256sum --check --status 2>/dev/null; then
    printf 'using cached artifact: %s\n' "$output"
    return
  fi
  rm -f "$output" "$partial"
  mkdir -p "$(dirname "$output")"
  curl --fail --location --retry 20 --retry-all-errors \
    --speed-limit 1024 --speed-time 180 --output "$partial" "$url"
  echo "$checksum  $partial" | sha256sum --check
  mv "$partial" "$output"
}

mkdir -p "$(dirname "$target")" "$wheelhouse" "$cuda_packages"
if ! echo "$model_checksum  $target" | sha256sum --check --status 2>/dev/null; then
  download_checked "$model_archive_checksum" "$model_url" "$model_cache"
  unzip -p "$model_cache" dpvo.pth > "$target.partial"
  echo "$model_checksum  $target.partial" | sha256sum --check
  mv "$target.partial" "$target"
fi
echo "$model_checksum  $target" | sha256sum --check

while read -r checksum filename url; do
  [[ -n $checksum && $checksum != \#* ]] || continue
  download_checked "$checksum" "$url" "$wheelhouse/$filename"
done < "$root/deployment/dpv_slam/python-wheels.sha256"

while read -r checksum filename; do
  [[ -n $checksum && $checksum != \#* ]] || continue
  download_checked "$checksum" \
    "https://conda.anaconda.org/nvidia/linux-64/$filename" \
    "$cuda_packages/$filename"
done < "$root/deployment/dpv_slam/cuda-build-packages.sha256"

printf 'DPV_SLAM_ARTIFACT_ROOT=%q\n' "$artifact_root"
