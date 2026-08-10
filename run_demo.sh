#!/usr/bin/env bash

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
image_name="${DALITI_DOCKER_IMAGE:-daliti:latest}"
startup_wait=5
drain_wait=8

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 /absolute/path/to/dataset.bag" >&2
    echo "Optional: set DALITI_DOCKER_IMAGE to use an image other than daliti:latest." >&2
    exit 2
fi

if ! bag_file="$(realpath "$1")" || [[ ! -f "${bag_file}" ]]; then
    echo "Bag file not found: $1" >&2
    exit 1
fi

if docker info >/dev/null 2>&1; then
    docker_cmd=(docker)
elif sudo -n docker info >/dev/null 2>&1; then
    docker_cmd=(sudo -n docker)
else
    echo "Docker is not accessible. Add the current user to the Docker group or configure passwordless sudo for Docker." >&2
    exit 1
fi

if ! "${docker_cmd[@]}" image inspect "${image_name}" >/dev/null 2>&1; then
    echo "Docker image not found: ${image_name}" >&2
    echo "Build it from the workspace root:" >&2
    echo "  docker build -t ${image_name} -f src/DaLiTI/Dockerfile ." >&2
    exit 1
fi

data_dir="$(dirname "${bag_file}")"
trajectory_dir="${repo_dir}/eskf_lio/Log"
run_id="demo_$(date +%Y%m%d_%H%M%S)_$$"
container_name="daliti_${run_id}"
tum_file="${trajectory_dir}/daliti_${run_id}.tum"
log_file="${trajectory_dir}/daliti_${run_id}.log"
mkdir -p "${trajectory_dir}"

bag_info="$("${docker_cmd[@]}" run --rm \
    -v "${bag_file}:/input.bag:ro" \
    "${image_name}" /bin/bash -lc \
    'source /opt/ros/noetic/setup.bash && rosbag info --yaml /input.bag')"
bag_start="$(awk '$1 == "start:" {print $2; exit}' <<<"${bag_info}")"
bag_end="$(awk '$1 == "end:" {print $2; exit}' <<<"${bag_info}")"
bag_duration="$(awk '$1 == "duration:" {print $2; exit}' <<<"${bag_info}")"
lidar_messages="$(awk '
    $1 == "-" && $2 == "topic:" {topic=$3}
    topic == "/ouster/points" && $1 == "messages:" {print $2; exit}
' <<<"${bag_info}")"
if [[ -z "${bag_start}" || -z "${bag_end}" || -z "${bag_duration}" ||
      -z "${lidar_messages}" ]]; then
    echo "Could not read bag timing or the /ouster/points message count." >&2
    exit 1
fi
play_timeout="$(awk -v duration="${bag_duration}" 'BEGIN {printf "%d", duration + 181}')"

xauth_file="${XAUTHORITY:-}"
if [[ -z "${xauth_file}" || ! -f "${xauth_file}" ]]; then
    for candidate in "/run/user/$(id -u)"/xauth_*; do
        if [[ -f "${candidate}" ]]; then
            xauth_file="${candidate}"
            break
        fi
    done
fi
if [[ -z "${xauth_file}" || ! -f "${xauth_file}" ]]; then
    echo "No Xauthority file was found; RViz cannot be started safely." >&2
    exit 1
fi

gpu_args=()
if command -v nvidia-smi >/dev/null 2>&1 &&
   "${docker_cmd[@]}" run --rm --gpus all \
       -e NVIDIA_VISIBLE_DEVICES=all \
       -e NVIDIA_DRIVER_CAPABILITIES=all \
       --entrypoint /bin/bash "${image_name}" -lc \
       'test -e /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0' >/dev/null 2>&1; then
    gpu_args=(
        --gpus all
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=all
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia
    )
    if [[ -d /dev/dri ]]; then
        gpu_args+=(--device /dev/dri:/dev/dri)
    fi
fi

container_active=0
cleanup() {
    if (( container_active == 1 )) &&
       "${docker_cmd[@]}" container inspect "${container_name}" >/dev/null 2>&1; then
        "${docker_cmd[@]}" logs "${container_name}" >"${log_file}" 2>&1 || true
        "${docker_cmd[@]}" stop -t 30 "${container_name}" >/dev/null 2>&1 || true
        "${docker_cmd[@]}" rm "${container_name}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting DaLiTI demo"
echo "  bag:   ${bag_file}"
echo "  image: ${image_name}"
echo "  run:   ${run_id}"

"${docker_cmd[@]}" run -d --name "${container_name}" --network host \
    "${gpu_args[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XAUTHORITY=/tmp/daliti.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${xauth_file}:/tmp/daliti.xauth:ro" \
    -v "${data_dir}:${data_dir}:ro" \
    -v "${trajectory_dir}:/results:rw" \
    "${image_name}" /bin/zsh -lc \
    "source /opt/ros/noetic/setup.zsh && source /ws/devel/setup.zsh && exec roslaunch daliti demo.launch rosbag_file_with_path:='${bag_file}' trajectory_output_dir:=/results trajectory_run_id:='${run_id}' disable_ros1_eol_warnings:=1" \
    >/dev/null
container_active=1

for ((check_index=0; check_index<120; ++check_index)); do
    if "${docker_cmd[@]}" exec "${container_name}" /bin/bash -lc \
        "source /opt/ros/noetic/setup.bash && rosservice list 2>/dev/null | grep -qx /play/pause_playback"; then
        break
    fi
    if [[ "$("${docker_cmd[@]}" inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null || true)" != "true" ]]; then
        echo "The DaLiTI container exited during startup. See ${log_file}." >&2
        exit 1
    fi
    if (( check_index == 119 )); then
        echo "The rosbag pause service did not become ready." >&2
        exit 1
    fi
    sleep 1
done

required_nodes=(
    /image_convert
    /rebvo_node
    /daliti_imuPreintegration
    /daliti_failureDetection
    /daliti_visual_feature
    /daliti_visual_odometry
    /daliti_visual_loop
    /feature_extract
    /laserMapping
)
for ((check_index=0; check_index<120; ++check_index)); do
    node_list="$("${docker_cmd[@]}" exec "${container_name}" /bin/bash -lc \
        "source /opt/ros/noetic/setup.bash && rosnode list 2>/dev/null")"
    missing_node=0
    for required_node in "${required_nodes[@]}"; do
        if ! grep -qx "${required_node}" <<<"${node_list}"; then
            missing_node=1
            break
        fi
    done
    if (( missing_node == 0 )); then
        break
    fi
    if (( check_index == 119 )); then
        echo "Not all required algorithm nodes became ready." >&2
        exit 1
    fi
    sleep 1
done

echo "All algorithm nodes are ready; starting the bag in ${startup_wait} seconds."
sleep "${startup_wait}"
unpause_result="$("${docker_cmd[@]}" exec "${container_name}" /bin/bash -lc \
    "source /opt/ros/noetic/setup.bash && rosservice call /play/pause_playback 'data: false'")"
if [[ "${unpause_result}" != *"success: True"* ]]; then
    echo "Failed to start rosbag playback: ${unpause_result}" >&2
    exit 1
fi

bag_finished=0
for ((check_index=0; check_index<play_timeout; ++check_index)); do
    if ! "${docker_cmd[@]}" exec "${container_name}" pgrep -x play >/dev/null 2>&1; then
        bag_finished=1
        break
    fi
    sleep 1
done
if (( bag_finished == 0 )); then
    echo "Rosbag playback timed out after ${play_timeout} seconds." >&2
    exit 1
fi

echo "Rosbag playback finished; draining pending callbacks."
sleep "${drain_wait}"
"${docker_cmd[@]}" stop -t 30 "${container_name}" >/dev/null || true
"${docker_cmd[@]}" logs "${container_name}" >"${log_file}" 2>&1 || true
"${docker_cmd[@]}" rm "${container_name}" >/dev/null || true
container_active=0

if [[ ! -s "${tum_file}" ]]; then
    echo "The TUM trajectory is missing or empty: ${tum_file}" >&2
    exit 1
fi
if [[ "$(stat -c %u "${tum_file}")" != "$(id -u)" ]]; then
    "${docker_cmd[@]}" run --rm \
        -v "${trajectory_dir}:/results:rw" \
        --entrypoint chown "${image_name}" \
        "$(id -u):$(id -g)" "/results/$(basename "${tum_file}")"
fi
if ! awk 'NR > 1 && $1 <= previous {exit 1} {previous=$1}' "${tum_file}"; then
    echo "The TUM timestamps are not strictly increasing." >&2
    exit 1
fi

tum_rows="$(wc -l <"${tum_file}")"
tum_first="$(awk 'NR == 1 {print $1; exit}' "${tum_file}")"
tum_last="$(awk 'END {print $1}' "${tum_file}")"
minimum_rows="$((lidar_messages * 99 / 100))"
if ! awk -v first="${tum_first}" -v last="${tum_last}" \
    -v start="${bag_start}" -v end="${bag_end}" \
    'BEGIN {exit !((first-start) <= 1.0 && (end-last) <= 1.0)}'; then
    echo "The TUM trajectory does not cover both ends of the bag." >&2
    exit 1
fi
if (( tum_rows < minimum_rows )); then
    echo "The TUM trajectory has ${tum_rows} rows; expected at least ${minimum_rows}." >&2
    exit 1
fi

coverage="$(awk -v first="${tum_first}" -v last="${tum_last}" \
    -v start="${bag_start}" -v end="${bag_end}" \
    'BEGIN {printf "%.6f", 100 * (last-first) / (end-start)}')"
echo "DaLiTI demo completed successfully."
echo "  trajectory: ${tum_file}"
echo "  log:        ${log_file}"
echo "  rows:       ${tum_rows}/${lidar_messages}"
echo "  coverage:   ${coverage}%"
