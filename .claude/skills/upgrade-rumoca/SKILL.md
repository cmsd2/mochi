---
name: upgrade-rumoca
description: Walk through upgrading mochi's pinned rumoca version. Use when the user asks to upgrade or test a new rumoca release (e.g. "upgrade to rumoca v0.X.Y", "let's move to the latest rumoca", "try rumoca v1.0"). Tags the current state, probes the new rumoca's CLI + JSON, migrates the loader on master, runs the version matrix as the gate, updates the README compatibility table, and cuts a new mochi tag.
---

# Upgrade mochi's pinned rumoca version

mochi pins to one rumoca version per tag (see `CLAUDE.md`, README's `## Compatibility`). Upgrades are forward-only — no backports, no runtime version dispatch. Each upgrade ends in a new mochi tag and a new row in the compatibility table.

## Order of operations

### 1. Tag the current state

Annotated tag at HEAD before any migration work, so the prior pin is discoverable. Bump mochi's minor (the upgrade is always a breaking change for users on the old rumoca).

```sh
git tag -a v0.X.Y -m "Pinned to rumoca v<old>"
```

### 2. Pick the target rumoca version

**Take the latest patch in the chosen release series**, not an earlier patch — earlier patches likely have bugs already fixed downstream. If the user says "v0.8", that means the latest v0.8.x (e.g. v0.8.12).

**Default to one minor at a time** (0.7 → 0.8 → 0.9), since each rumoca minor tends to change CLI and/or JSON shape and chaining migrations compounds unknowns. **But probe first** — if step 4 finds the intermediate series is a dead end (the JSON dump dropped fields mochi needs, no emit mode restores them), skip it with eyes open and target the next series instead. Record the skipped series as its own row in the compatibility table with a one-line note on why. The skip is information for the next maintainer.

### 3. Build the target binary

```sh
make rumoca-matrix RUMOCA_VERSIONS=v<new>
```

This builds (with cache) under `.rumoca-cache/v<new>/rumoca` and runs `mxpm test -y` against it. The first run captures the failure-mode baseline. Don't try to fix anything yet — read the failures to confirm whether it's a CLI break, a JSON break, or both.

### 4. Probe the new rumoca shape

Confirm the working CLI form by running the binary directly:

```sh
.rumoca-cache/v<new>/rumoca --help
.rumoca-cache/v<new>/rumoca compile --help  # if subcommands exist
```

Then diff the JSON output for the four representative models (each exercises a different loader path):

- `examples/RLCircuit.mo` — basic linear DAE, parameters, residuals, `der()`.
- `examples/BouncingBall.mo` — events (`when` clauses → `:c` / `:fc` / `:fr`).
- `examples/extends/RCFilterExtends.mo` — connectors, `extends` inheritance, instance namespaces.
- `examples/msl/PIDClosedLoop.mo` — MSL, calculated parameters, `:fx_init` bindings, `If`-gated branches.

For each model, run with both the old and new rumoca and look for differences in:

- Top-level JSON keys (e.g. `fx` vs `f_x`, presence/absence of `:u` / `:c` / `:fc` / `:fr` / `:fx_init`)
- AST node tags (e.g. `Terminal` vs `Literal`, `ComponentReference` vs `VarRef`, `FunctionCall` shape)
- Variable-info shape (`causality`, `variability`, `start` field)
- Event extraction shape (`:fz` list vs `:fc` + `:fr` dicts)

A `python3 -c '... json.load ...'` snippet that walks unique top-level dict tags is the fastest way to scope the delta.

### 5. Plan the loader migration

Concrete list of changes needed in `mochi-loader.lisp`. If only the CLI changed, the diff is ~5 lines in `mochi--run-rumoca`. If JSON shape changed too, scope:

- `mochi--ast-to-maxima` dispatch table
- `mochi--terminal-to-maxima` / literal parsing
- `mochi--function-call-to-maxima` / builtin dispatch
- `mochi--cref-name` / `mochi--cref-to-maxima` / `mochi--bare-cref-name`
- `mochi--simple-residual` (residual = lhs - rhs)
- `mochi--build-init-bindings` and `mochi--start-value` (fx_init fallback)
- `mochi--extract-events` / `mochi--cond-to-events`
- top-level key names in `$mod_load`

Document the plan as `TaskCreate` items so progress is visible.

### 6. Implement on master

No long-lived migration branch — the old version stays accessible via the tag from step 1. The current head moves forward.

### 7. Test

Iterate with `mxpm test -y` (fast) until the suite goes green. Then run the matrix as the **gate**:

```sh
make rumoca-matrix RUMOCA_VERSIONS=v<new>
```

Only proceed if the matrix passes (give or take rumoca-side regressions that are out of mochi's hands — record those, don't try to work around them).

### 8. Update the README compatibility table

Add a new row to `## Compatibility` in README.md naming the new mochi tag, the new rumoca version, and a short note on the CLI form (so users glancing at the table can verify their setup).

Also update any stale CLI / flag references in the README body (e.g. the `## How it works` diagram, prerequisite snippets, docstring examples).

### 9. Commit + tag

One commit per upgrade. Title is concrete about the version pin: `Migrate loader to rumoca v0.X.Y`. Body bullets the loader changes by category (CLI / AST / events / etc.).

After committing, cut the new mochi tag:

```sh
git tag -a v0.<NEXT>.0 -m "Pinned to rumoca v<new>"
```

Don't push without asking — the user decides when to publish.

## Anti-patterns

- **Bisecting across release series.** Don't try to find "the highest rumoca that works without changes." Pick the target series's latest patch and migrate; the matrix proves the result.
- **Probing on assumption rather than evidence.** Always run step 4 (JSON shape diff) on the candidate series before planning the migration. v0.7.28 → v0.8.x was a richness *regression* — v0.8 dropped causality, inputs, events, and `:fx_init` from its JSON dump entirely. Without the probe, the migration would have looked tractable from the CLI alone.
- **Carrying parallel code paths for old rumoca versions.** The loader stays linear — one shape, one CLI. Users on old rumoca install the matching older mochi tag.
- **Working around rumoca-side regressions.** If rumoca's behaviour change is genuinely a regression (e.g. silently dropping a feature mochi relied on), record the test failure and consider whether to skip the upgrade. Don't add code in mochi to compensate.
- **Skipping the tag in step 1.** Without it, the old pin becomes hard to find six months later.
