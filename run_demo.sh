#!/usr/bin/env bash
# Run DaLiTI on a rosbag inside Docker, write the TUM trajectory and the log, and optionally evaluate it.

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
startup_wait=5
drain_wait=8

usage() {
    cat >&2 <<EOF
Usage: $0 [options] /path/to/dataset.bag

Options:
  --profile NAME     preset for a sequence (default: daliti-indoor-smoke)
                       daliti-indoor-smoke  whole bag, camera calibration hitsz
                       daliti-outdoor-smoke whole prepared bag, camera calibration daliti-rooftop
                       rooftop       whole raw bag, camera calibration daliti-rooftop, /thermal/cam1/image8bit
  --start SEC        start playback SEC seconds after the bag start (default: 0 for every profile)
  --duration SEC     play only SEC seconds (default: to the end of the bag)
  --camera NAME      VINS camera calibration daliti/config/NAME_camera.yaml
  --gt FILE          ground-truth TUM trajectory: report the ATE after the run
  --image NAME       Docker image (default: daliti:<git commit>, built automatically if missing)
  --rebuild          rebuild the Docker image before running
  -h, --help         show this help

The trajectory (TUM, stamped with the state time) and the log are written to eskf_lio/Log/.
Environment: DALITI_DOCKER_IMAGE and DALITI_CAMERA_CONFIG are used if --image / --camera are not given;
http_proxy / https_proxy are passed to the image build if set.
EOF
}

profile="daliti-indoor-smoke"
opt_start="" opt_duration="" opt_camera="" gt_arg="" opt_image="" rebuild=0 bag_arg=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) profile="${2:?}"; shift 2 ;;
        --start) opt_start="${2:?}"; shift 2 ;;
        --duration) opt_duration="${2:?}"; shift 2 ;;
        --camera) opt_camera="${2:?}"; shift 2 ;;
        --gt) gt_arg="${2:?}"; shift 2 ;;
        --image) opt_image="${2:?}"; shift 2 ;;
        --rebuild) rebuild=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "Unknown option: $1" >&2; usage; exit 2 ;;
        *)
            if [[ -n "${bag_arg}" ]]; then echo "Only one bag can be given." >&2; usage; exit 2; fi
            bag_arg="$1"; shift ;;
    esac
done
if [[ -z "${bag_arg}" ]]; then usage; exit 2; fi

case "${profile}" in
    daliti-indoor-smoke) p_camera="" ;;
    daliti-outdoor-smoke|rooftop) p_camera="daliti-rooftop" ;;
    *) echo "Unknown profile: ${profile} (daliti-indoor-smoke | daliti-outdoor-smoke | rooftop)" >&2; exit 2 ;;
esac
play_start="${opt_start:-0}"
play_duration="${opt_duration}"
camera_config="${opt_camera:-${DALITI_CAMERA_CONFIG:-${p_camera}}}"
input_config="${repo_dir}/daliti/config/${profile}_input.yaml"
if [[ ! -f "${input_config}" ]]; then
    echo "Input configuration not found: ${input_config}" >&2
    exit 1
fi
is_number() { [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; }
if ! is_number "${play_start}" || { [[ -n "${play_duration}" ]] && ! is_number "${play_duration}"; }; then
    echo "--start / --duration must be non-negative numbers of seconds." >&2
    exit 2
fi
if [[ -n "${camera_config}" && ! -f "${repo_dir}/daliti/config/${camera_config}_camera.yaml" ]]; then
    echo "Camera calibration not found: daliti/config/${camera_config}_camera.yaml" >&2
    exit 1
fi

if ! bag_file="$(realpath "${bag_arg}")" || [[ ! -f "${bag_file}" ]]; then
    echo "Bag file not found: ${bag_arg}" >&2
    exit 1
fi
gt_file=""
if [[ -n "${gt_arg}" ]]; then
    if ! gt_file="$(realpath "${gt_arg}")" || [[ ! -f "${gt_file}" ]]; then
        echo "Ground-truth file not found: ${gt_arg}" >&2
        exit 1
    fi
fi

if docker info >/dev/null 2>&1; then
    docker_cmd=(docker)
elif sudo -n docker info >/dev/null 2>&1; then
    docker_cmd=(sudo -n docker)
else
    echo "Docker is not accessible. Add the current user to the Docker group or configure passwordless sudo for Docker." >&2
    exit 1
fi

# Image: by default tagged with the git commit, so the image always matches the checked-out code.
git_commit=""
if git -C "${repo_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git_commit="$(git -C "${repo_dir}" rev-parse --short=10 HEAD)"
fi
if [[ -n "${opt_image}" ]]; then
    image_name="${opt_image}"
elif [[ -n "${DALITI_DOCKER_IMAGE:-}" ]]; then
    image_name="${DALITI_DOCKER_IMAGE}"
elif [[ -n "${git_commit}" ]]; then
    image_name="daliti:${git_commit}"
else
    image_name="daliti:latest"
fi

build_image() {
    if [[ -z "${git_commit}" ]]; then
        echo "Cannot build ${image_name} automatically: ${repo_dir} is not a git checkout." >&2
        echo "Build it from the workspace root: docker build -t ${image_name} -f src/DaLiTI/Dockerfile ." >&2
        exit 1
    fi
    if [[ -n "$(git -C "${repo_dir}" status --porcelain --untracked-files=no)" ]]; then
        echo "Warning: uncommitted changes are not part of the image; it is built from commit ${git_commit}." >&2
    fi
    local build_args=(--network host)
    local proxy_var
    for proxy_var in http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY; do
        if [[ -n "${!proxy_var:-}" ]]; then
            build_args+=(--build-arg "${proxy_var}=${!proxy_var}")
        fi
    done
    echo "Building Docker image ${image_name} from commit ${git_commit} (this takes several minutes) ..."
    local attempt
    for attempt in 1 2 3; do
        if git -C "${repo_dir}" archive --format=tar --prefix=src/DaLiTI/ HEAD |
            "${docker_cmd[@]}" build "${build_args[@]}" -t "${image_name}" -f src/DaLiTI/Dockerfile -; then
            return 0
        fi
        if (( attempt < 3 )); then
            echo "The image build failed (attempt ${attempt} of 3; network errors are often transient). Retrying in 15 s ..." >&2
            sleep 15
        fi
    done
    echo "Could not build ${image_name}. Check the network / proxy (http_proxy, https_proxy) and run again." >&2
    exit 1
}
if (( rebuild == 1 )) || ! "${docker_cmd[@]}" image inspect "${image_name}" >/dev/null 2>&1; then
    build_image
fi

# One run per host: the container uses the host network, i.e. one ROS master port shared by all runs. A second run
# would register nodes with the same names and shut the first one down.
active_run="$("${docker_cmd[@]}" ps --format '{{.Names}}' --filter 'name=daliti_demo_' | head -1)"
if [[ -n "${active_run}" ]]; then
    echo "Another DaLiTI run is active (container ${active_run}); wait for it to finish or stop it." >&2
    exit 1
fi
if (exec 3<>/dev/tcp/127.0.0.1/11311) 2>/dev/null; then
    echo "A ROS master is already running on port 11311; stop it first (the container uses the host network)." >&2
    exit 1
fi

data_dir="$(dirname "${bag_file}")"
trajectory_dir="${repo_dir}/eskf_lio/Log"
run_id="demo_$(date +%Y%m%d_%H%M%S)_$$"
container_name="daliti_${run_id}"
tum_file="${trajectory_dir}/daliti_${run_id}.tum"
log_file="${trajectory_dir}/daliti_${run_id}.log"
eval_file="${trajectory_dir}/daliti_${run_id}_eval.json"
mkdir -p "${trajectory_dir}"

bag_info="$("${docker_cmd[@]}" run --rm \
    -v "${bag_file}:/input.bag:ro" \
    -v "${input_config}:/daliti_input.yaml:ro" \
    "${image_name}" /bin/bash -lc \
    'source /opt/ros/noetic/setup.bash && rosbag info --yaml /input.bag && python3 -c '\''import yaml; print("input_image_topic: " + yaml.safe_load(open("/daliti_input.yaml"))["img_topic"])'\''')"
image_topic="$(awk '$1 == "input_image_topic:" {print $2; exit}' <<<"${bag_info}")"
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
image_topic_messages="$(awk -v topic="${image_topic}" '
    $1 == "-" && $2 == "topic:" {current=$3}
    current == topic && $1 == "messages:" {print $2; exit}
' <<<"${bag_info}")"
if [[ -z "${image_topic_messages}" ]]; then
    echo "Warning: the bag has no ${image_topic} topic; check img_topic in ${input_config}." >&2
fi

# Playback window (absolute bag time) and the number of LiDAR frames expected in it.
read -r window_start window_end expected_frames < <(awk -v s="${bag_start}" -v e="${bag_end}" -v d="${bag_duration}" \
    -v off="${play_start}" -v len="${play_duration}" -v n="${lidar_messages}" 'BEGIN {
        ws = s + off; we = (len == "") ? e : ws + len; if (we > e) we = e;
        printf "%.6f %.6f %d\n", ws, we, (we > ws) ? n * (we - ws) / d : 0 }')
if ! awk -v ws="${window_start}" -v we="${window_end}" 'BEGIN {exit !(we - ws > 2.0)}'; then
    echo "The playback window is empty or too short (start ${play_start} s, bag duration ${bag_duration} s)." >&2
    exit 1
fi
play_timeout="$(awk -v ws="${window_start}" -v we="${window_end}" 'BEGIN {printf "%d", we - ws + 181}')"

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

launch_args="rosbag_file_with_path:='${bag_file}' trajectory_output_dir:=/results trajectory_run_id:='${run_id}'"
launch_args+=" disable_ros1_eol_warnings:=1 bag_start:=${play_start} input_config:=/daliti_input.yaml"
if [[ -n "${play_duration}" ]]; then
    launch_args+=" bag_duration:=${play_duration}"
fi
if [[ -n "${camera_config}" ]]; then
    launch_args+=" camera_config:=${camera_config}"
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
echo "  bag:     ${bag_file}"
echo "  profile: ${profile} (window ${play_start} s + ${play_duration:-all}, camera ${camera_config:-hitsz}, image ${image_topic})"
echo "  image:   ${image_name}"
echo "  run:     ${run_id}"

"${docker_cmd[@]}" run -d --name "${container_name}" --network host \
    "${gpu_args[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XAUTHORITY=/tmp/daliti.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${xauth_file}:/tmp/daliti.xauth:ro" \
    -v "${data_dir}:${data_dir}:ro" \
    -v "${trajectory_dir}:/results:rw" \
    -v "${input_config}:/daliti_input.yaml:ro" \
    "${image_name}" /bin/zsh -lc \
    "source /opt/ros/noetic/setup.zsh && source /ws/devel/setup.zsh && exec roslaunch daliti demo.launch ${launch_args}" \
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
play_misses=0
for ((check_index=0; check_index<play_timeout; ++check_index)); do
    if "${docker_cmd[@]}" exec "${container_name}" pgrep -x play >/dev/null 2>&1; then
        play_misses=0
    else
        # A single failed `docker exec` is not proof that playback ended: require three consecutive misses.
        play_misses=$((play_misses + 1))
        if (( play_misses >= 3 )); then
            bag_finished=1
            break
        fi
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
minimum_rows="$((expected_frames * 99 / 100))"
status=0
if ! awk -v first="${tum_first}" -v last="${tum_last}" \
    -v start="${window_start}" -v end="${window_end}" \
    'BEGIN {exit !((first-start) <= 1.5 && (end-last) <= 1.5)}'; then
    echo "The TUM trajectory does not cover both ends of the playback window." >&2
    status=1
fi
if (( tum_rows < minimum_rows )); then
    echo "The TUM trajectory has ${tum_rows} rows; expected at least ${minimum_rows}." >&2
    status=1
fi
coverage="$(awk -v first="${tum_first}" -v last="${tum_last}" \
    -v start="${window_start}" -v end="${window_end}" \
    'BEGIN {printf "%.6f", 100 * (last-first) / (end-start)}')"

eval_mounts=(-v "${trajectory_dir}:/results:rw" -v "${repo_dir}/tools:/tools:ro")
eval_args=("/results/$(basename "${tum_file}")" --json "/results/$(basename "${eval_file}")")
if [[ -n "${gt_file}" ]]; then
    eval_mounts+=(-v "$(dirname "${gt_file}"):/gt:ro")
    eval_args+=(--gt "/gt/$(basename "${gt_file}")")
fi
echo "Evaluation:"
if ! "${docker_cmd[@]}" run --rm "${eval_mounts[@]}" "${image_name}" python3 /tools/evaluate_tum.py "${eval_args[@]}"; then
    echo "The evaluation failed." >&2
    status=1
fi
if [[ -f "${eval_file}" && "$(stat -c %u "${eval_file}")" != "$(id -u)" ]]; then
    "${docker_cmd[@]}" run --rm -v "${trajectory_dir}:/results:rw" --entrypoint chown "${image_name}" \
        "$(id -u):$(id -g)" "/results/$(basename "${eval_file}")" || true
fi

if (( status == 0 )); then
    echo "DaLiTI demo completed successfully."
else
    echo "DaLiTI demo finished with errors (see above)."
fi
echo "  trajectory: ${tum_file}"
echo "  log:        ${log_file}"
echo "  evaluation: ${eval_file}"
echo "  rows:       ${tum_rows}/${expected_frames}"
echo "  coverage:   ${coverage}%"
exit "${status}"
