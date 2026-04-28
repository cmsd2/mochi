#!/usr/bin/env python3
"""Convert rumoca's JSON DAE output into a Maxima-loadable .mac file.

Usage:
    rumoca_to_mac.py <model.mo> [--rumoca-bin PATH]

Emits Maxima source on stdout with these definitions:

    mod__model_name : "RLCircuit"$
    mod__params : [[R, 1.0], [L, 0.5], ...] $        /* name + default value */
    mod__states : [iL, vC] $                          /* state-variable symbols */
    mod__derivs : [der_iL, der_vC] $                  /* der(state) symbols */
    mod__inputs : [Vin] $                             /* input symbols */
    mod__outputs : [y] $                              /* output symbols */
    mod__equations : [L * der_iL - (Vin - R*iL - vC),
                      Ccap * der_vC - iL,
                      y - vC] $                       /* residual form: each = 0 */

The mod_load() Maxima function then assembles these into a model struct.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from typing import Any


def find_rumoca() -> str:
    explicit = os.environ.get("RUMOCA_BIN")
    if explicit:
        return explicit
    found = shutil.which("rumoca")
    if found:
        return found
    home_cargo = os.path.expanduser("~/.cargo/bin/rumoca")
    if os.path.exists(home_cargo):
        return home_cargo
    sys.exit("ERROR: rumoca not found. Install via 'cargo install rumoca' or set RUMOCA_BIN.")


def run_rumoca(rumoca: str, mo_path: str) -> dict[str, Any]:
    cwd = os.path.dirname(os.path.abspath(mo_path)) or "."
    name = os.path.basename(mo_path)
    proc = subprocess.run(
        [rumoca, "compile", "--json", name],
        capture_output=True,
        text=True,
        cwd=cwd,
    )
    if proc.returncode != 0:
        sys.exit(f"rumoca failed:\n{proc.stderr}")
    return json.loads(proc.stdout)


# ----------------------------------------------------------------------------
# Lightweight scan of the .mo file for `input Real X;` and `output Real X;`.
# rumoca's --json output doesn't tag inputs/outputs explicitly, but they're
# easy to spot in the source.
# ----------------------------------------------------------------------------

INPUT_RE = re.compile(r"\binput\s+(?:Real|Integer|Boolean)\s+([A-Za-z_]\w*)")
OUTPUT_RE = re.compile(r"\boutput\s+(?:Real|Integer|Boolean)\s+([A-Za-z_]\w*)")


def scan_io(mo_path: str) -> tuple[list[str], list[str]]:
    text = open(mo_path).read()
    # Strip block and line comments before scanning to avoid false hits.
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return INPUT_RE.findall(text), OUTPUT_RE.findall(text)


# ----------------------------------------------------------------------------
# AST → Maxima-syntax expression
# rumoca's expression nodes are tagged dicts: {"Binary": {...}}, {"VarRef": {...}}, ...
# ----------------------------------------------------------------------------

# Operator tag → Maxima infix
BINARY_OPS = {
    "Add": "+",
    "Sub": "-",
    "Mul": "*",
    "Div": "/",
    "Pow": "^",
    "Eq": "=",
    "And": "and",
    "Or": "or",
    "Gt": ">",
    "Lt": "<",
    "Geq": ">=",
    "Leq": "<=",
    "Neq": "#",
}

UNARY_OPS = {
    "Neg": "-",
    "Pos": "+",
    "Not": "not ",
}


def node_to_maxima(node: Any) -> str:
    """Convert one rumoca AST node to Maxima source."""
    if not isinstance(node, dict):
        raise ValueError(f"Unexpected non-dict node: {node!r}")

    if len(node) != 1:
        raise ValueError(f"Expected single-tag node, got keys: {list(node)}")
    tag, body = next(iter(node.items()))

    if tag == "VarRef":
        return body["name"]

    if tag == "Literal":
        # body is { "Real": 1.0 } or { "Integer": 0 } or { "Boolean": true } or { "String": "..." }
        if not isinstance(body, dict) or len(body) != 1:
            return repr(body)
        ltype, lval = next(iter(body.items()))
        if ltype == "Real":
            # Always emit with decimal, never trailing dot
            s = repr(float(lval))
            return s
        if ltype == "Integer":
            return str(int(lval))
        if ltype == "Boolean":
            return "true" if lval else "false"
        if ltype == "String":
            return f'"{lval}"'
        return repr(lval)

    if tag == "Binary":
        op_tag = next(iter(body["op"]))
        op_str = BINARY_OPS.get(op_tag)
        if op_str is None:
            raise ValueError(f"Unknown binary op: {op_tag}")
        lhs = node_to_maxima(body["lhs"])
        rhs = node_to_maxima(body["rhs"])
        return f"({lhs}) {op_str} ({rhs})"

    if tag == "Unary":
        op_tag = next(iter(body["op"]))
        op_str = UNARY_OPS.get(op_tag)
        if op_str is None:
            raise ValueError(f"Unknown unary op: {op_tag}")
        arg = node_to_maxima(body["arg"])
        return f"({op_str}({arg}))"

    if tag == "BuiltinCall":
        fn = body["function"]
        args = [node_to_maxima(a) for a in body.get("args", [])]
        # Translate Modelica builtin names to Maxima
        if fn == "Der":
            # der(x) becomes the symbol der_x
            if len(args) != 1:
                raise ValueError(f"Der with {len(args)} args")
            inner = args[0].strip().strip("()")
            return f"der_{inner}"
        # Trig/transcendental: Modelica capitalises, Maxima doesn't
        name_map = {
            "Sin": "sin", "Cos": "cos", "Tan": "tan",
            "Asin": "asin", "Acos": "acos", "Atan": "atan", "Atan2": "atan2",
            "Sinh": "sinh", "Cosh": "cosh", "Tanh": "tanh",
            "Exp": "exp", "Log": "log", "Log10": "log10",
            "Sqrt": "sqrt", "Abs": "abs",
            "Min": "min", "Max": "max",
            "Floor": "floor", "Ceil": "ceiling",
        }
        return f"{name_map.get(fn, fn.lower())}({', '.join(args)})"

    if tag == "FunctionCall":
        name = body.get("name", "?")
        args = [node_to_maxima(a) for a in body.get("args", [])]
        return f"{name}({', '.join(args)})"

    if tag == "Paren":
        return f"({node_to_maxima(body['expr'])})"

    raise ValueError(f"Unsupported AST tag: {tag!r}")


def equation_to_residual(eq_entry: dict) -> str:
    """rumoca stores equations as residuals (lhs - rhs = 0).
    The 'residual' field is a Binary{Sub, lhs, rhs} expression."""
    return node_to_maxima(eq_entry["residual"])


# ----------------------------------------------------------------------------
# Top-level: emit Maxima source
# ----------------------------------------------------------------------------

def emit(mo_path: str, model: dict, inputs: list[str], outputs: list[str]) -> str:
    out: list[str] = []
    out.append(f"/* Auto-generated from {os.path.basename(mo_path)} by rumoca_to_mac.py */")
    out.append("")

    # Try to recover a model name from the .mo file (rumoca JSON omits it)
    name_match = re.search(r"\b(?:model|class|block)\s+([A-Za-z_]\w*)", open(mo_path).read())
    model_name = name_match.group(1) if name_match else "Unnamed"

    # Kill any prior bindings of the symbols used by this model — user variables
    # named the same as a model parameter (e.g. `A2`) would otherwise substitute
    # into our params/residuals.
    symbols = (
        list(model.get("p", {}))
        + list(model.get("x", {}))
        + [f"der_{s}" for s in model.get("x", {})]
        + list(inputs)
        + list(outputs)
    )
    if symbols:
        out.append(f"kill({', '.join(symbols)})$")

    out.append(f'mod__model_name : "{model_name}"$')

    # Parameters
    params = model.get("p", {})
    pnames = list(params)
    pdefaults = []
    for name, info in params.items():
        start = info.get("start", {}).get("Literal", {})
        if "Real" in start:
            val = repr(float(start["Real"]))
        elif "Integer" in start:
            val = str(int(start["Integer"]))
        elif "Boolean" in start:
            val = "true" if start["Boolean"] else "false"
        else:
            val = "0"
        pdefaults.append(f"[{name}, {val}]")
    out.append(f"mod__params : [{', '.join(pdefaults)}]$")

    # States and their derivative symbols
    states = list(model.get("x", {}))
    out.append(f"mod__states : [{', '.join(states)}]$")
    out.append(f"mod__derivs : [{', '.join(f'der_{s}' for s in states)}]$")

    # Initial conditions
    ics: list[str] = []
    for name, info in model.get("x", {}).items():
        start = info.get("start", {}).get("Literal", {})
        if "Real" in start:
            val = repr(float(start["Real"]))
        elif "Integer" in start:
            val = str(int(start["Integer"]))
        else:
            val = "0"
        ics.append(f"[{name}, {val}]")
    out.append(f"mod__initial : [{', '.join(ics)}]$")

    # Inputs and outputs from the source-text scan
    out.append(f"mod__inputs : [{', '.join(inputs)}]$")
    out.append(f"mod__outputs : [{', '.join(outputs)}]$")

    # Equations as residuals (each = 0)
    eq_strs = [equation_to_residual(e) for e in model.get("f_x", [])]
    out.append("mod__residuals : [")
    for i, e in enumerate(eq_strs):
        suffix = "," if i < len(eq_strs) - 1 else ""
        out.append(f"  {e}{suffix}")
    out.append("]$")

    return "\n".join(out) + "\n"


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main(argv: list[str]) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mo_path")
    parser.add_argument("--rumoca-bin", default=None)
    args = parser.parse_args(argv)

    rumoca = args.rumoca_bin or find_rumoca()
    model = run_rumoca(rumoca, args.mo_path)
    inputs, outputs = scan_io(args.mo_path)
    sys.stdout.write(emit(args.mo_path, model, inputs, outputs))


if __name__ == "__main__":
    main(sys.argv[1:])
