#!/usr/bin/env python3
"""Scans raw OSM data for places where road merging (build::fuse_chain, gated by
ModelBuilder::build_glue_map in src/model_builder.cpp) is likely to silently carry one way's
turn:lanes-derived restriction across a boundary into an adjacent way that never had that
restriction (or had a different one) -- the root cause behind a real violation
check_turn_lane_routing.py found in durlacher-tor.osm (way 653779041's through-only lane routing
to a right turn and a near-U-turn 56m past where that tag was ever meaningful).

Why this can't be checked from the .xodr output alone: lane_plan_differs (src/model_builder.cpp)
-- the function that decides whether a merge boundary needs its own <laneSection> -- compares
only lane type/width/roadmark, never turn_directions. So when two adjacent ways have the same
lane shape but different turn:lanes coverage, they merge into one continuous laneSection with no
visible boundary in the emitted XML: the only place this is still visible is the *source* OSM
tags, before merging erases the seam. This script looks for exactly that seam.

Method (a deliberately approximate mirror of the C++ topology/merge logic, not a byte-exact
replica -- see caveats below):
  1. Parse every OSM way that would be extracted as a road (osm::is_road_way's own highway-class
     allowlist, mirrored here).
  2. Find nodes touched by exactly two road ways, both as an *endpoint* (first/last node) and by
     no third way as an *interior* point -- an approximation of "not a junction" (real degree
     computation also depends on topology splits this script does not replicate).
  3. Exclude nodes tagged as a feature-split point (traffic_signals/stop/give_way/traffic_sign/
     crossing=traffic_signals) -- those always get their own road boundary regardless of lane
     shape, so they're never at risk.
  4. Compare the two ways' base forward/backward lane counts (mirroring infer_lanes' own
     derivation, turn:lanes-count-override included). If they differ, a lane-count change would
     already force its own boundary (or a merge-chain taper) -- not at risk.
  5. Compare each way's turn:lanes-derived per-lane restriction signature. Flag the boundary if
     they differ and at least one side has a real (non-empty) restriction.

Caveats (why this is a *candidate* list for manual review, not a verdict):
  - Direction (oneway=-1, or a reversed way relative to its neighbor) is not accounted for --
    forward/backward car-lane restrictions are compared positionally, not re-oriented.
  - Width/roadmark equivalence is approximated by comparing the *raw* width/lane_markings tag
    strings, not the derived per-lane numeric width lane_plan_differs actually compares -- this
    errs toward under-reporting (treating two differently-tagged-but-numerically-equal widths as
    "already segmented, not at risk") rather than over-reporting.
  - Whether the resulting junction/geometry actually produces a *visible* routing violation
    depends on the angle at wherever the chain eventually reaches a junction, which this script
    does not evaluate -- that's what check_turn_lane_routing.py does, on the actual .xodr output.

Usage:
    python3 test/check_merge_turn_lane_consistency.py examples/*.osm
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_turn_lane_routing import decode_turn_lanes, is_oneway  # reuse, don't reimplement

ALLOWED_HIGHWAY = {
    'motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'residential',
    'motorway_link', 'trunk_link', 'primary_link', 'secondary_link', 'tertiary_link',
    'living_street', 'service', 'road', 'busway', 'construction',
}


def is_road_way(tags):
    if tags.get('highway') not in ALLOWED_HIGHWAY:
        return False
    if tags.get('area') == 'yes':
        return False
    if tags.get('access') == 'private':
        return False
    return True


def is_feature_split_node(tags):
    if tags.get('highway') in ('traffic_signals', 'stop', 'give_way'):
        return True
    if 'traffic_sign' in tags:
        return True
    if tags.get('crossing') == 'traffic_signals':
        return True
    return False


def parse_osm(path):
    data = Path(path).read_text(encoding='utf-8', errors='replace')
    node_tags = {}
    for nm in re.finditer(r"<node\b([^>]*?)(/>|>(.*?)</node>)", data, re.S):
        attrs = dict(re.findall(r"""(\w+)=['"]([^'"]*)['"]""", nm.group(1)))
        nid = attrs.get('id')
        if not nid:
            continue
        body = nm.group(3) or ''
        node_tags[nid] = dict(re.findall(r"""<tag k=['"]([^'"]+)['"] v=['"]([^'"]*)['"]""", body))

    ways = {}
    for wm in re.finditer(r"""<way id=['"](\d+)['"][^>]*>(.*?)</way>""", data, re.S):
        wid, body = wm.group(1), wm.group(2)
        nds = re.findall(r"""<nd ref=['"](\d+)['"]""", body)
        tags = dict(re.findall(r"""<tag k=['"]([^'"]+)['"] v=['"]([^'"]*)['"]""", body))
        ways[wid] = {'nodes': nds, 'tags': tags}
    return node_tags, ways


def int_tag(tags, key):
    v = tags.get(key)
    if v is None:
        return None
    try:
        return int(v.strip())
    except ValueError:
        return None


def base_lane_counts(tags):
    """Mirrors infer_lanes' forward/backward count derivation (src/infer.cpp), turn:lanes-count
    override included, so two ways' effective lane *shape* can be compared the way
    lane_plan_differs would (approximately -- width/roadmark are compared separately below)."""
    oneway = is_oneway(tags)
    total = int_tag(tags, 'lanes')
    if total is None:
        total = 1 if oneway else 2
    total = max(1, total)
    f = int_tag(tags, 'lanes:forward')
    b = int_tag(tags, 'lanes:backward')
    if oneway:
        fwd, bwd = (f if f is not None else total), 0
    elif f is not None or b is not None:
        fwd = f if f is not None else max(1, total - (b if b is not None else 1))
        bwd = b if b is not None else max(1, total - fwd)
    elif total == 1:
        fwd, bwd = 1, 1
    else:
        fwd, bwd = total // 2, total // 2

    decoded_fwd = decode_turn_lanes(tags, '' if oneway else ':forward')
    decoded_back = None if oneway else decode_turn_lanes(tags, ':backward')
    if decoded_fwd is not None and decoded_fwd['car_slot_positions']:
        fwd = len(decoded_fwd['car_slot_positions'])
    if decoded_back is not None and decoded_back['car_slot_positions']:
        bwd = len(decoded_back['car_slot_positions'])
    return fwd, bwd


def width_shape(tags):
    """Raw-string proxy for the width/roadmark side of lane_plan_differs -- see module caveats."""
    return tuple(tags.get(k, '') for k in ('width', 'width:lanes', 'width:lanes:forward',
                                            'width:lanes:backward', 'lane_markings', 'highway'))


def restriction_signature(tags):
    oneway = is_oneway(tags)
    decoded_fwd = decode_turn_lanes(tags, '' if oneway else ':forward')
    decoded_back = None if oneway else decode_turn_lanes(tags, ':backward')
    if decoded_fwd is None and decoded_back is None:
        return None
    fwd_sets = tuple(tuple(s) for s in (decoded_fwd['turn_sets'] if decoded_fwd else []))
    back_sets = tuple(tuple(s) for s in (decoded_back['turn_sets'] if decoded_back else []))
    return (fwd_sets, back_sets)


def scan(osm_path):
    node_tags, ways = parse_osm(osm_path)
    road_ways = {wid: w for wid, w in ways.items() if is_road_way(w['tags'])}

    endpoint_touches = defaultdict(list)  # node -> [way_id, ...] touching as first/last node
    interior_touches = defaultdict(set)   # node -> {way_id, ...} touching as an interior point
    for wid, w in road_ways.items():
        nds = w['nodes']
        if len(nds) < 2:
            continue
        endpoint_touches[nds[0]].append(wid)
        endpoint_touches[nds[-1]].append(wid)
        for n in nds[1:-1]:
            interior_touches[n].add(wid)

    findings = []
    for node, touching_ways in endpoint_touches.items():
        if len(touching_ways) != 2:
            continue
        if interior_touches.get(node):
            continue  # a third way passes through here: not a plain 2-degree point
        if is_feature_split_node(node_tags.get(node, {})):
            continue
        wid_a, wid_b = touching_ways
        if wid_a == wid_b:
            continue
        tags_a, tags_b = road_ways[wid_a]['tags'], road_ways[wid_b]['tags']
        if base_lane_counts(tags_a) != base_lane_counts(tags_b):
            continue  # a real lane-count change gets its own boundary/taper regardless
        if width_shape(tags_a) != width_shape(tags_b):
            continue  # a width/type/roadmark change already forces its own boundary
        sig_a, sig_b = restriction_signature(tags_a), restriction_signature(tags_b)
        if sig_a is None and sig_b is None:
            continue  # neither way tags turn:lanes: nothing to silently lose
        if sig_a == sig_b:
            continue  # identical on both sides: nothing changes across the boundary
        findings.append({
            'node': node, 'way_a': wid_a, 'way_b': wid_b,
            'name_a': tags_a.get('name', ''), 'name_b': tags_b.get('name', ''),
            'sig_a': sig_a, 'sig_b': sig_b,
        })
    return findings


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        sys.exit(2)
    total = 0
    for path in paths:
        findings = scan(path)
        total += len(findings)
        print(f"=== {path}: {len(findings)} candidate boundary(ies) ===")
        for f in findings:
            print(f"  node {f['node']}: way {f['way_a']} ({f['name_a']!r}) -- "
                  f"way {f['way_b']} ({f['name_b']!r})")
            print(f"    way {f['way_a']} restriction: {f['sig_a']}")
            print(f"    way {f['way_b']} restriction: {f['sig_b']}")
    print(f"\n{total} candidate boundary(ies) total across {len(paths)} file(s).")
    sys.exit(0 if total == 0 else 1)


if __name__ == '__main__':
    main()
