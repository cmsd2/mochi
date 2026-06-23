#!/usr/bin/env bash
# rumoca-matrix.sh — build a set of rumoca versions from a local clone
# and run the mochi test suite against each, then print a summary matrix.
#
# Usage:
#   scripts/rumoca-matrix.sh                       # default version set
#   scripts/rumoca-matrix.sh v0.9.7 v0.9.8         # custom versions
#
# Env overrides:
#   RUMOCA_SRC    path to a CogniPilot/rumoca checkout (default: ../rumoca
#                 relative to this repo)
#   RUMOCA_CACHE  where to cache built binaries (default: .rumoca-cache
#                 inside this repo)
#
# Each version is materialised as its own git worktree under
# $RUMOCA_CACHE/<tag>/src (so the source repo's working tree stays put),
# built with `cargo build --release --bin rumoca`, and the binary is
# cached at $RUMOCA_CACHE/<tag>/rumoca.  Re-runs only rebuild versions
# whose cached binary is missing.
#
# Disk: each version's target/ directory is ~hundreds of MB; the cache
# can grow to a few GB across a typical version set.  Wipe with
# `make clean-rumoca-matrix` (or just `rm -rf .rumoca-cache`).

set -euo pipefail

mochi_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rumoca_src="${RUMOCA_SRC:-$(cd -- "$mochi_dir/.." && pwd)/rumoca}"
rumoca_cache="${RUMOCA_CACHE:-$mochi_dir/.rumoca-cache}"

if [ "$#" -gt 0 ]; then
  versions=("$@")
else
  versions=(v0.7.28 v0.8.12 v0.9.8)
fi

if [ ! -d "$rumoca_src/.git" ]; then
  printf 'rumoca-matrix: source repo not found at %s\n' "$rumoca_src" >&2
  printf '  set RUMOCA_SRC=/path/to/rumoca to override\n' >&2
  exit 2
fi

mkdir -p "$rumoca_cache"

# Build a rumoca binary at $rumoca_cache/<tag>/rumoca.  Idempotent —
# does nothing if the binary already exists.
build_version() {
  local tag="$1"
  local outdir="$rumoca_cache/$tag"
  local outbin="$outdir/rumoca"

  if [ -x "$outbin" ]; then
    printf '  cached: %s\n' "$outbin" >&2
    return 0
  fi

  if ! git -C "$rumoca_src" rev-parse --verify --quiet "$tag" >/dev/null; then
    printf '  ERROR: tag %s not found in %s\n' "$tag" "$rumoca_src" >&2
    return 1
  fi

  local work="$outdir/src"
  if [ ! -d "$work" ]; then
    printf '  creating worktree at %s\n' "$work" >&2
    git -C "$rumoca_src" worktree add --detach "$work" "$tag" >&2
  fi

  printf '  building %s (release)...\n' "$tag" >&2
  (cd "$work" && cargo build --release --bin rumoca) >&2

  mkdir -p "$outdir"
  cp "$work/target/release/rumoca" "$outbin"
  printf '  built: %s\n' "$outbin" >&2
}

# Run the mochi test suite with RUMOCA_BIN set, stream all output.
run_tests() {
  local bin="$1"
  cd "$mochi_dir"
  RUMOCA_BIN="$bin" mxpm test -y 2>&1 || true
}

summary_pattern='(rtest_mochi[a-z_]*\.mac: [0-9]+/[0-9]+|^Tests (passed|failed):)'

declare -a summary_rows
for tag in "${versions[@]}"; do
  printf '\n=== rumoca %s ===\n' "$tag"
  if ! build_version "$tag"; then
    summary_rows+=("$tag|BUILD FAILED")
    continue
  fi

  bin="$rumoca_cache/$tag/rumoca"
  printf '  running mochi tests...\n'
  out=$(run_tests "$bin")

  # Stream the per-file pass/fail lines so the user sees progress.
  echo "$out" | grep -E "$summary_pattern" || true

  # Final aggregate line — what we put in the matrix.
  total_line=$(echo "$out" | grep -E '^Tests (passed|failed):' | tail -1)
  summary_rows+=("$tag|${total_line:-(no summary emitted — likely a load error; see full output above)}")
done

printf '\n===== matrix summary =====\n'
printf '%-12s  %s\n' 'version' 'result'
printf '%-12s  %s\n' '-------' '------'
for row in "${summary_rows[@]}"; do
  IFS='|' read -r v s <<<"$row"
  printf '%-12s  %s\n' "$v" "$s"
done
