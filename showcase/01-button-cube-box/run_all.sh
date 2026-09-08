#!/usr/bin/env bash
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(cd "$TEST_DIR/../.." && pwd)/text2mujoco_codex"
MUJOCO_PYTHON="${MUJOCO_PYTHON:-python3}"
RENDER_BACKEND="${MUJOCO_RENDER_BACKEND:-auto}"
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-/tmp/text2mujoco-pycache}"

cd "$TEST_DIR"
mkdir -p output/static output/screenshots

"$MUJOCO_PYTHON" "$SKILL_DIR/scripts/validate_scene_spec.py" scene_spec.json --json \
  | tee output/static/spec_validation.json
"$MUJOCO_PYTHON" contract_test.py | tee output/static/contract_test.json
"$MUJOCO_PYTHON" validator_unit_test.py | tee output/static/validator_unit_test.json
MUJOCO_GL=disable "$MUJOCO_PYTHON" runtime_probe.py \
  | tee output/runtime_probe.json
MUJOCO_GL=disable "$MUJOCO_PYTHON" physics_smoke.py \
  --output-dir output --result output/physics_results.json

run_render() {
  local backend="$1"
  MUJOCO_GL="$backend" "$MUJOCO_PYTHON" render_smoke.py \
    --screenshot-dir output/screenshots \
    --result "output/render_${backend}_results.json"
}

if [[ "$RENDER_BACKEND" == "auto" && "$(uname -s)" == "Darwin" ]]; then
  if command -v mjpython >/dev/null 2>&1; then
    MUJOCO_GL=glfw mjpython render_smoke.py \
      --screenshot-dir output/screenshots \
      --result output/render_glfw_results.json | tee output/render_glfw.log
  else
    run_render glfw | tee output/render_glfw.log
  fi
  cp output/render_glfw_results.json output/render_results.json
  printf 'Render backend: glfw (macOS native CGL context)\n'
elif [[ "$RENDER_BACKEND" == "auto" ]]; then
  set +e
  run_render egl >output/render_egl.log 2>&1
  egl_status=$?
  set -e
  if [[ $egl_status -eq 0 ]]; then
    cp output/render_egl_results.json output/render_results.json
    printf 'Render backend: egl\n'
  else
    printf 'EGL failed with exit %s; retrying in a fresh OSMesa process.\n' "$egl_status"
    run_render osmesa | tee output/render_osmesa.log
    cp output/render_osmesa_results.json output/render_results.json
    printf 'Render backend: osmesa\n'
  fi
else
  run_render "$RENDER_BACKEND" | tee "output/render_${RENDER_BACKEND}.log"
  cp "output/render_${RENDER_BACKEND}_results.json" output/render_results.json
  printf 'Render backend: %s\n' "$RENDER_BACKEND"
fi

printf '\nArtifacts:\n'
find output -maxdepth 4 -type f -print | sort
