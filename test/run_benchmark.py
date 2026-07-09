#!/usr/bin/env python3
"""Aggregate evaluation harness for osm2xodr.

Runs the osm2xodr binary over every fixture in a manifest (see
benchmark_manifest.json), under one or more CLI flag variants, and aggregates:

  - wall-clock time and (best-effort, via `/usr/bin/time`) peak RSS,
  - output size and road/junction/connector counts parsed from the .xodr,
  - the four existing geometric self-consistency checkers
    (check_junction_continuity.py, check_turn_lane_routing.py,
    check_road_link_continuity.py, check_road_geometry_continuity.py), and
  - a connector-fallback rate (fraction of junction <connection> entries whose
    connectingRoad is a direct road-to-road link rather than a real synthetic
    connector -- see README "Important limitations").

into a CSV (one row per fixture x variant) and a Markdown summary table
suitable for pasting into a paper's evaluation section.

This is a thin orchestration layer, not a replacement for the checkers: it
shells out to them and parses their existing human-readable summary line with
a regex, the same way the checkers themselves parse .xodr XML with regexes
rather than a real parser. If a checker's summary-line wording changes, update
the corresponding *_RE pattern below.

Usage:
    python3 test/run_benchmark.py --binary build/osm2xodr
    python3 test/run_benchmark.py --binary build/osm2xodr --variants default,no-junction-merge
    python3 test/run_benchmark.py --binary build/osm2xodr --fixtures durlacher-tor,karl-wilhelm
    python3 test/run_benchmark.py --binary build/osm2xodr --strict   # nonzero exit on any default-variant mismatch
"""
import argparse
import csv
import json
import platform
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

JUNCTION_RE = re.compile(r'Checked (\d+) junction connector lane links, (\d+) mismatched')
ROADLINK_RE = re.compile(r'Checked (\d+) plain road-to-road lane link\(s\), (\d+) mismatched')
TURNLANE_RE = re.compile(
    r'Checked (\d+) tagged way\(s\) with lane-level turn restrictions '
    r'\((\d+) lane/junction-end checks\), (\d+) violation'
)
GEOMCONT_RE = re.compile(r'Checked (\d+) internal geometry-to-geometry boundary\(ies\), (\d+) mismatched')
# A checker's mismatch tolerance (1mm/0.001rad) is calibrated for geometry the converter
# constructs itself (junction connectors); it is not a meaningful bug signal at plain
# road-to-road boundaries, where the two adjacent OSM ways' own digitized headings rarely
# line up to sub-degree precision. So in addition to the pass/fail count, pull the actual
# per-mismatch dpos/dhdg magnitudes out of each checker's own diagnostic lines and report
# their distribution -- a few cm / a couple of degrees is source-data noise; tens of degrees
# or meters is a real defect.
DPOS_DHDG_RE = re.compile(r'dpos=([\d.]+)m dhdg=([\d.]+)deg')
BOUNDS_RE = re.compile(
    r'''<bounds\s+minlat=['"]([-\d.]+)['"]\s+minlon=['"]([-\d.]+)['"]\s+maxlat=['"]([-\d.]+)['"]\s+maxlon=['"]([-\d.]+)['"]''')
NODE_RE = re.compile(r'''<node\b[^>]*\blat=['"]([-\d.]+)['"][^>]*\blon=['"]([-\d.]+)['"]''')


def origin_for_osm(osm_path):
    """Bounding-box center if present, else the first node's coordinate."""
    text = Path(osm_path).read_text(encoding='utf-8', errors='replace')
    m = BOUNDS_RE.search(text)
    if m:
        minlat, minlon, maxlat, maxlon = (float(g) for g in m.groups())
        return (minlat + maxlat) / 2.0, (minlon + maxlon) / 2.0
    m = NODE_RE.search(text)
    if m:
        return float(m.group(1)), float(m.group(2))
    raise ValueError(f"{osm_path}: no <bounds> or <node> found to derive a projection origin")


def parse_rss_bytes(stderr_text, system):
    if system == 'Darwin':
        m = re.search(r'(\d+)\s+maximum resident set size', stderr_text)
        if m:
            return int(m.group(1))  # BSD/macOS time -l reports bytes
    else:
        m = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', stderr_text)
        if m:
            return int(m.group(1)) * 1024  # GNU time -v reports kB
    return None


def run_timed(cmd):
    """Runs cmd, wrapped in /usr/bin/time for peak RSS when available.

    Returns (returncode, wall_seconds, peak_rss_bytes_or_None, stdout, stderr).
    Peak-RSS capture is best-effort and POSIX-only; it silently degrades to
    wall-time-only if /usr/bin/time is missing or its output can't be parsed.
    """
    system = platform.system()
    flag = '-l' if system == 'Darwin' else '-v'
    wrapped = ['/usr/bin/time', flag] + cmd
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(wrapped, capture_output=True, text=True)
        wall = time.perf_counter() - t0
        rss = parse_rss_bytes(proc.stderr, system)
        return proc.returncode, wall, rss, proc.stdout, proc.stderr
    except FileNotFoundError:
        t0 = time.perf_counter()
        proc = subprocess.run(cmd, capture_output=True, text=True)
        wall = time.perf_counter() - t0
        return proc.returncode, wall, None, proc.stdout, proc.stderr


def road_junction_map(xodr_text):
    """id -> junction attribute for every <road>, using the same tolerant
    attribute-order-independent parsing idiom as the checker scripts."""
    m = {}
    for rm in re.finditer(r'<road\b([^>]*)>', xodr_text):
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', rm.group(1)))
        if 'id' in attrs:
            m[attrs['id']] = attrs.get('junction', '-1')
    return m


def xodr_stats(xodr_path):
    text = Path(xodr_path).read_text(encoding='utf-8', errors='replace')
    rj = road_junction_map(text)
    n_roads = sum(1 for j in rj.values() if j == '-1')
    n_connectors = sum(1 for j in rj.values() if j != '-1')
    n_junctions = len(re.findall(r'<junction\b[^>]*>', text))

    total_conn = 0
    fallback_conn = 0
    for conn_m in re.finditer(r'<connection\b([^>]*)>', text):
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', conn_m.group(1)))
        cr = attrs.get('connectingRoad')
        total_conn += 1
        if rj.get(cr, '-1') == '-1':
            fallback_conn += 1

    return {
        'xodr_bytes': Path(xodr_path).stat().st_size,
        'n_roads': n_roads,
        'n_connector_roads': n_connectors,
        'n_junctions': n_junctions,
        'n_connections': total_conn,
        'n_fallback_connections': fallback_conn,
        'fallback_rate': (fallback_conn / total_conn) if total_conn else None,
    }


def run_checker(args):
    proc = subprocess.run([sys.executable] + args, capture_output=True, text=True)
    return proc.returncode == 0, proc.stdout, proc.stderr


def mismatch_distribution(checker_stdout):
    """Percentiles of dpos(m)/dhdg(deg) across every mismatch line a checker printed.

    Returns None if there were no mismatch lines (either everything matched, or the
    checker found nothing to check), else a dict of summary statistics.
    """
    pairs = DPOS_DHDG_RE.findall(checker_stdout)
    if not pairs:
        return None
    dpos = sorted(float(p[0]) for p in pairs)
    dhdg = sorted(float(p[1]) for p in pairs)

    def pct(sorted_vals, p):
        if not sorted_vals:
            return None
        idx = min(len(sorted_vals) - 1, int(round(p * (len(sorted_vals) - 1))))
        return sorted_vals[idx]

    return {
        'n': len(pairs),
        'dpos_median_m': pct(dpos, 0.5), 'dpos_p95_m': pct(dpos, 0.95), 'dpos_max_m': dpos[-1],
        'dhdg_median_deg': pct(dhdg, 0.5), 'dhdg_p95_deg': pct(dhdg, 0.95), 'dhdg_max_deg': dhdg[-1],
    }


def evaluate_fixture(binary, fixture, variant_name, variant_flags, outdir, repeat, left_hand):
    osm_path = REPO_ROOT / fixture['osm']
    lat0, lon0 = origin_for_osm(osm_path)

    tag = f"{fixture['name']}__{variant_name}"
    xodr_path = outdir / f"{tag}.xodr"
    report_path = outdir / f"{tag}_report.txt"

    cmd = [str(binary), str(osm_path), str(xodr_path),
           '--name', fixture['name'],
           '--origin-lat', f'{lat0:.7f}', '--origin-lon', f'{lon0:.7f}',
           '--report', str(report_path)] + list(variant_flags)
    if left_hand:
        cmd.append('--left-hand-traffic')

    wall_times, rss_values = [], []
    returncode, stderr = 0, ''
    for _ in range(max(1, repeat)):
        returncode, wall, rss, _stdout, stderr = run_timed(cmd)
        wall_times.append(wall)
        if rss is not None:
            rss_values.append(rss)

    row = {
        'fixture': fixture['name'], 'tier': fixture.get('tier', 'junction'),
        'tags': ';'.join(fixture.get('tags', [])), 'variant': variant_name,
        'convert_ok': returncode == 0,
        'wall_s_median': statistics.median(wall_times),
        'peak_rss_mb': (statistics.median(rss_values) / (1024 * 1024)) if rss_values else None,
    }
    if returncode != 0:
        row['error'] = stderr.strip().splitlines()[-1] if stderr.strip() else 'osm2xodr exited nonzero'
        return row

    row.update(xodr_stats(xodr_path))

    lh_args = ['--left-hand-traffic'] if left_hand else []

    ok_j, out_j, _ = run_checker([str(REPO_ROOT / 'test/check_junction_continuity.py'), str(xodr_path)] + lh_args)
    m = JUNCTION_RE.search(out_j)
    row['junction_checked'], row['junction_mismatched'] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    row['junction_check_ok'] = ok_j
    for k, v in (mismatch_distribution(out_j) or {}).items():
        row[f'junction_{k}'] = v

    ok_r, out_r, _ = run_checker([str(REPO_ROOT / 'test/check_road_link_continuity.py'), str(xodr_path)] + lh_args)
    m = ROADLINK_RE.search(out_r)
    row['roadlink_checked'], row['roadlink_mismatched'] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    row['roadlink_check_ok'] = ok_r
    for k, v in (mismatch_distribution(out_r) or {}).items():
        row[f'roadlink_{k}'] = v

    ok_t, out_t, _ = run_checker(
        [str(REPO_ROOT / 'test/check_turn_lane_routing.py'), str(osm_path), str(xodr_path)] + lh_args)
    m = TURNLANE_RE.search(out_t)
    if m:
        row['turnlane_checked_ways'], row['turnlane_checks'], row['turnlane_violations'] = (
            int(m.group(1)), int(m.group(2)), int(m.group(3)))
    else:
        row['turnlane_checked_ways'] = row['turnlane_checks'] = row['turnlane_violations'] = None
    row['turnlane_check_ok'] = ok_t

    ok_g, out_g, _ = run_checker([str(REPO_ROOT / 'test/check_road_geometry_continuity.py'), str(xodr_path)])
    m = GEOMCONT_RE.search(out_g)
    row['geomcont_checked'], row['geomcont_mismatched'] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    row['geomcont_check_ok'] = ok_g

    row['all_checks_ok'] = ok_j and ok_r and ok_t and ok_g
    return row


def write_csv(rows, path):
    fields = []
    for r in rows:
        for k in r:
            if k not in fields:
                fields.append(k)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def write_markdown(rows, path):
    lines = ['# osm2xodr benchmark summary', '']
    by_variant = {}
    for r in rows:
        by_variant.setdefault(r['variant'], []).append(r)

    for variant, vrows in by_variant.items():
        lines.append(f'## Variant: `{variant}`')
        lines.append('')
        lines.append('| fixture | tier | roads | junctions | fallback rate | '
                      'junction mismatches | roadlink mismatches (p95 dpos/dhdg) | '
                      'turnlane violations | geomcont mismatches | wall (s) | peak RSS (MB) |')
        lines.append('| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |')
        for r in vrows:
            if not r.get('convert_ok'):
                lines.append(f"| {r['fixture']} | {r['tier']} | FAILED: {r.get('error', '')} | | | | | | | | |")
                continue
            fb = r.get('fallback_rate')
            fb_s = f"{fb:.1%}" if fb is not None else 'n/a'
            rss = r.get('peak_rss_mb')
            rss_s = f"{rss:.1f}" if rss is not None else 'n/a'
            rl_p95 = ''
            if r.get('roadlink_dpos_p95_m') is not None:
                rl_p95 = f" (p95 {r['roadlink_dpos_p95_m']:.3f}m / {r['roadlink_dhdg_p95_deg']:.2f}deg, " \
                         f"max {r['roadlink_dpos_max_m']:.3f}m / {r['roadlink_dhdg_max_deg']:.2f}deg)"
            lines.append(
                f"| {r['fixture']} | {r['tier']} | {r['n_roads']} | {r['n_junctions']} | {fb_s} | "
                f"{r['junction_mismatched']}/{r['junction_checked']} | "
                f"{r['roadlink_mismatched']}/{r['roadlink_checked']}{rl_p95} | "
                f"{r['turnlane_violations']}/{r['turnlane_checks']} | "
                f"{r['geomcont_mismatched']}/{r['geomcont_checked']} | "
                f"{r['wall_s_median']:.3f} | {rss_s} |")
        lines.append('')

        n = len(vrows)
        ok = sum(1 for r in vrows if r.get('all_checks_ok'))
        lines.append(f"{ok}/{n} fixtures passed all self-consistency checks with zero mismatches/violations "
                      f"under `{variant}`.")
        lines.append('')
        lines.append("Note: the road-link checker's 1mm/0.001rad tolerance is calibrated for geometry the "
                      "converter constructs itself (junction connectors); at plain road-to-road boundaries "
                      "between two independently-digitized OSM ways it will report a 'mismatch' for any "
                      "ordinary source-data kink, so treat the mismatch *count* here as close to meaningless "
                      "on real-world extracts and look at the p95/max dpos/dhdg instead -- a few cm / a few "
                      "degrees is source noise (see README limitations); values far larger than that at a "
                      "specific boundary are worth a manual look.")
        lines.append('')

    Path(path).write_text('\n'.join(lines), encoding='utf-8')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', required=True, help='Path to the osm2xodr executable')
    ap.add_argument('--manifest', default=str(REPO_ROOT / 'test/benchmark_manifest.json'))
    ap.add_argument('--outdir', default=str(REPO_ROOT / 'test/benchmark_out'))
    ap.add_argument('--fixtures', help='Comma-separated fixture names to run (default: all in manifest)')
    ap.add_argument('--variants', help='Comma-separated flag-variant names to run (default: all in manifest)')
    ap.add_argument('--repeat', type=int, default=1, help='Timing repeats per fixture x variant (median reported)')
    ap.add_argument('--left-hand-traffic', action='store_true')
    ap.add_argument('--csv-out', default=None)
    ap.add_argument('--md-out', default=None)
    ap.add_argument('--strict', action='store_true',
                     help="Exit nonzero if any fixture fails to convert or fails a self-consistency check "
                          "under the 'default' variant")
    args = ap.parse_args()

    manifest = json.loads(Path(args.manifest).read_text(encoding='utf-8'))
    fixtures = manifest['fixtures']
    variants = manifest['flag_variants']

    if args.fixtures:
        wanted = set(args.fixtures.split(','))
        fixtures = [f for f in fixtures if f['name'] in wanted]
    if args.variants:
        wanted_v = set(args.variants.split(','))
        variants = {k: v for k, v in variants.items() if k in wanted_v}

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    rows = []
    for fixture in fixtures:
        for variant_name, variant_flags in variants.items():
            print(f"[{fixture['name']} / {variant_name}] converting...", file=sys.stderr)
            row = evaluate_fixture(args.binary, fixture, variant_name, variant_flags, outdir,
                                    args.repeat, args.left_hand_traffic)
            rows.append(row)

    csv_out = args.csv_out or str(outdir / 'results.csv')
    md_out = args.md_out or str(outdir / 'results.md')
    write_csv(rows, csv_out)
    write_markdown(rows, md_out)
    print(f"Wrote {csv_out} and {md_out}", file=sys.stderr)

    if args.strict:
        default_rows = [r for r in rows if r['variant'] == 'default']
        bad = [r for r in default_rows if not r.get('convert_ok') or not r.get('all_checks_ok', False)]
        if bad:
            print(f"STRICT: {len(bad)}/{len(default_rows)} default-variant fixture(s) failed:", file=sys.stderr)
            for r in bad:
                print(f"  {r['fixture']}: {r.get('error', 'check mismatch/violation')}", file=sys.stderr)
            sys.exit(1)


if __name__ == '__main__':
    main()
