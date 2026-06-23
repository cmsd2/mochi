# mochi

## rumoca compatibility policy

mochi pins to a single rumoca version per tag. The README's `## Compatibility` table is authoritative — read it before assuming any specific version.

- **Forward-only.** No backports, no runtime version dispatch in the loader.
- **A new row in the table is added only after `make rumoca-matrix RUMOCA_VERSIONS=v0.X.Y` passes** against the candidate rumoca. The matrix script (`scripts/rumoca-matrix.sh`) builds tagged binaries from a local rumoca clone (default `../rumoca/`) and runs `mxpm test -y` against each.
- When rumoca cuts a new release, propose the upgrade as a new mochi tag rather than patching the current pin.

Why pinned-and-forward: rumoca reshapes CLI and JSON schema almost every minor release (CLI alone changed three times across v0.7.28 → v0.8.12 → v0.9.8). Supporting multiple versions in-process would mean parallel code paths that rot fast.

## Tests

`mxpm test -y` runs the suite. `rtest_mochi_msl.mac` needs the Modelica Standard Library reachable via `MODELICAPATH` or one of the auto-discovered paths (`$OPENMODELICAHOME/lib/omlibrary/Modelica*/` or `~/.modelica/library/<lib>/`); the file header documents this.
