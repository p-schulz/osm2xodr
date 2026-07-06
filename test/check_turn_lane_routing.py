#!/usr/bin/env python3
"""Verify turn:lanes-derived per-lane routing against the actual junction connections osm2xodr
generated, for every OSM way that carries a turn:lanes (or turn:lanes:forward/:backward) tag.

This is a companion to check_junction_continuity.py: that script verifies connector *geometry*
(position/heading continuity); this one verifies *semantic* routing -- that a lane tagged, say,
turn:lanes slot "left" only ever appears as the "from" lane of movements that actually turn left
geometrically, never straight-ahead or right-turn movements, and vice-versa.

It independently re-derives, from the raw OSM tags, the same {lane_id -> allowed turn buckets}
mapping that infer::decode_turn_lanes / infer_lanes (src/main.cpp) computes, then independently
re-derives, from the XODR geometry, the same {signed heading delta -> turn bucket} classification
that build::turn_bucket_for_delta (src/main.cpp) computes for each actual movement at a junction --
and cross-checks them.

Only covers OSM ways that survive as a *primary* road in the output (id "w<wayid>_<n>"); a way
fully absorbed as a non-primary/interior segment during road merging or compound-junction
clustering is reported separately as "skipped" rather than silently ignored.

Usage:
    python3 check_turn_lane_routing.py <file.osm> <file.xodr> [--left-hand-traffic]

Exit code is 0 if every checked lane's actual destinations are all within its tagged allowed
buckets (extra permissiveness from unrestricted lanes elsewhere is fine), 1 if any lane routed to
a bucket its tag disallows.
"""
import math
import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_junction_continuity import parse_xodr, lane_centerline_at_s, angle_diff, road_point_at_s

# Matches build::classification_direction_away_from_junction's lookahead (src/main.cpp): the
# immediate first-micro-segment tangent (what lane_centerline_at_s/the connector's own geometry
# uses) badly under-represents a road that keeps curving well past its first short sub-segment, so
# turn-bucket classification looks further down the road's own reference line instead.
CLASSIFICATION_LOOKAHEAD = 15.0


def classification_heading_away(road, at_start, lookahead):
    reach = min(lookahead, road['length'])
    origin_s = 0.0 if at_start else road['length']
    target_s = origin_s + reach if at_start else origin_s - reach
    ox, oy, _ = road_point_at_s(road, origin_s)
    tx, ty, _ = road_point_at_s(road, target_s)
    dx, dy = tx - ox, ty - oy
    if math.hypot(dx, dy) < 1e-6:
        _, _, h = road_point_at_s(road, origin_s)
        return h if at_start else h + math.pi
    return math.atan2(dy, dx)


def classification_heading_into(road, at_start, lookahead):
    return classification_heading_away(road, at_start, lookahead) + math.pi


def split_pipe_preserve_empty(s):
    return s.split('|')


def parse_turn_tokens(raw):
    tokens = []
    for t in raw.split(';'):
        l = t.strip().lower()
        if l == '' or l == 'none':
            continue
        tokens.append(l)
    return tokens


def decode_turn_lanes(tags, suffix):
    turn_raw = tags.get('turn:lanes' + suffix)
    if turn_raw is None:
        return None
    turn_slots = split_pipe_preserve_empty(turn_raw)
    access_raw = tags.get('access:lanes' + suffix)
    vehicle_raw = tags.get('vehicle:lanes' + suffix)
    access_slots = split_pipe_preserve_empty(access_raw) if access_raw is not None else []
    vehicle_slots = split_pipe_preserve_empty(vehicle_raw) if vehicle_raw is not None else []

    car_slot_positions = []
    turn_sets = []
    for i, slot in enumerate(turn_slots):
        excluded = (i < len(access_slots) and access_slots[i].strip().lower() == 'no') or \
                   (i < len(vehicle_slots) and vehicle_slots[i].strip().lower() == 'no')
        if excluded:
            continue
        car_slot_positions.append(i)
        turn_sets.append(parse_turn_tokens(slot))
    return {'car_slot_positions': car_slot_positions, 'turn_sets': turn_sets}


def is_oneway(tags):
    highway = tags.get('highway', '')
    oneway = tags.get('oneway')
    if oneway is not None:
        l = oneway.strip().lower()
        if l in ('-1', 'yes', 'true', '1'):
            return True
        if l in ('no', 'false', '0'):
            return False
    if tags.get('junction') in ('roundabout', 'circular'):
        return True
    return highway in ('motorway', 'motorway_link')


def lane_turn_directions(tags, left_hand):
    """Replicates infer_lanes' RHT/LHT lane-id assignment + turn_directions plumbing.
    Returns dict: lane_id (OpenDRIVE, signed) -> list of allowed tokens (empty = unrestricted)."""
    oneway = is_oneway(tags)
    decoded_fwd = decode_turn_lanes(tags, '' if oneway else ':forward')
    decoded_back = None if oneway else decode_turn_lanes(tags, ':backward')
    if decoded_fwd is None and decoded_back is None:
        return None

    result = {}

    def assign(decoded, sign):
        if decoded is None:
            return
        for i, turn_set in enumerate(decoded['turn_sets'], start=1):
            result[sign * i] = turn_set

    if not left_hand:
        assign(decoded_back, +1)   # left.: id +i
        assign(decoded_fwd, -1)    # right: id -i
    else:
        assign(decoded_fwd, +1)
        assign(decoded_back, -1)
    return result


def parse_osm_ways(path):
    data = open(path, encoding='utf-8').read()
    ways = {}
    for m in re.finditer(r'<way id="(\d+)"[^>]*>(.*?)</way>', data, re.S):
        wid = m.group(1)
        body = m.group(2)
        tags = dict(re.findall(r'<tag k="([^"]*)" v="([^"]*)"', body))
        ways[wid] = tags
    return ways


TURN_BUCKETS_ORDER = ['sharp_left', 'left', 'slight_left', 'through', 'slight_right', 'right', 'sharp_right']


def turn_bucket_for_delta(delta_rad):
    deg = math.degrees(delta_rad)
    if deg > 135.0:
        return 'sharp_left'
    if deg > 45.0:
        return 'left'
    if deg > 20.0:
        return 'slight_left'
    if deg >= -20.0:
        return 'through'
    if deg >= -45.0:
        return 'slight_right'
    if deg >= -135.0:
        return 'right'
    return 'sharp_right'


def bucket_allowed(tokens, bucket, signed_delta):
    if not tokens:
        return True
    for token in tokens:
        if token == bucket:
            return True
        if token == 'merge_to_left' and bucket == 'slight_left':
            return True
        if token == 'merge_to_right' and bucket == 'slight_right':
            return True
        if token == 'left' and bucket == 'slight_left':
            return True
        if token == 'right' and bucket == 'slight_right':
            return True
        if token == 'reverse' and ((bucket == 'sharp_left') if signed_delta >= 0.0 else (bucket == 'sharp_right')):
            return True
    return False


def road_end_touching_junction(road, junction_id):
    """Returns 's' (0.0 or road length) of whichever end of `road` links to junction_id, or None."""
    pred = road['pred']
    succ = road['succ']
    if pred and pred.get('elementType') == 'junction' and pred.get('elementId') == junction_id:
        return 0.0
    if succ and succ.get('elementType') == 'junction' and succ.get('elementId') == junction_id:
        return road['length']
    return None


def check(osm_path, xodr_path, left_hand):
    ways = parse_osm_ways(osm_path)
    data = open(xodr_path, encoding='utf-8').read()
    roads = parse_xodr(data)

    # way_id -> {lane_id: tokens}, only for ways whose tags actually decode turn:lanes.
    tagged = {}
    for wid, tags in ways.items():
        mapping = lane_turn_directions(tags, left_hand)
        if mapping is not None:
            tagged[wid] = mapping

    # Only keep ways where at least one lane has a non-empty restriction (an all-unrestricted
    # decode is not interesting -- nothing to violate).
    tagged = {wid: m for wid, m in tagged.items() if any(toks for toks in m.values())}

    checked_ways = 0
    skipped_ways = []
    total_lane_checks = 0
    violations = []

    road_id_re = re.compile(r'^w(\d+)_\d+$')
    # road id -> way id, for primary (unmerged-into or first-of-chain) roads only.
    primary_road_for_way = {}
    for rid, road in roads.items():
        m = road_id_re.match(rid)
        if not m:
            continue
        wid = m.group(1)
        if wid in tagged:
            primary_road_for_way.setdefault(wid, []).append(rid)

    for wid, lane_map in tagged.items():
        matches = primary_road_for_way.get(wid, [])
        if not matches:
            skipped_ways.append(wid)
            continue
        checked_ways += 1
        for rid in matches:
            road = roads[rid]
            for junction_id in set(x for x in [
                road['pred']['elementId'] if road['pred'] and road['pred'].get('elementType') == 'junction' else None,
                road['succ']['elementId'] if road['succ'] and road['succ'].get('elementType') == 'junction' else None,
            ] if x):
                s_at_junction = road_end_touching_junction(road, junction_id)
                if s_at_junction is None:
                    continue
                at_road_end_is_start = (s_at_junction == 0.0)

                for lane_id, tokens in lane_map.items():
                    if not tokens:
                        continue  # unrestricted lane: nothing to violate
                    if not any(lane_id in ls[1] for ls in road['lane_sections']):
                        continue
                    # Confirm the lane actually exists in the section applicable at this end (the
                    # lane_map here is derived from the *whole way's* tags, which for a merged road
                    # may not match the specific laneSection that's actually applicable at this end,
                    # e.g. lane_id existed in some other section of the merge chain) -- skip rather
                    # than crash in that case. The heading itself comes from the road's own
                    # reference-line classification direction (see classification_heading_into),
                    # matching what build_model's turn-bucket classification actually used, not the
                    # lane's own immediate-tangent travel heading.
                    try:
                        lane_centerline_at_s(roads, rid, lane_id, s_at_junction, left_hand)
                    except KeyError:
                        continue
                    in_hdg = classification_heading_into(road, at_road_end_is_start, CLASSIFICATION_LOOKAHEAD)

                    # Find every <connection> in this junction whose incomingRoad is `rid` (at the
                    # matching contact point) and from-lane == lane_id.
                    junction_body_m = re.search(r'<junction\b[^>]*\bid="' + re.escape(junction_id) +
                                                 r'"[^>]*>(.*?)</junction>', data, re.S)
                    if not junction_body_m:
                        continue
                    junction_body = junction_body_m.group(1)
                    total_lane_checks += 1
                    seen_any = False
                    for conn_m in re.finditer(r'<connection\b([^>]*)>(.*?)</connection>', junction_body, re.S):
                        cattrs = dict(re.findall(r'(\w+)="([^"]*)"', conn_m.group(1)))
                        if cattrs.get('incomingRoad') != rid:
                            continue
                        connecting_road = cattrs['connectingRoad']
                        for from_id, to_id in re.findall(r'<laneLink\s+from="(-?\d+)"\s+to="(-?\d+)"', conn_m.group(2)):
                            if int(from_id) != lane_id:
                                continue
                            seen_any = True
                            cr = roads[connecting_road]
                            to_id = int(to_id)
                            if cr['attrs'].get('junction', '-1') != '-1':
                                # Real synthetic connector: classification targets whatever real
                                # road it actually leads to (its own successor), matching build_model
                                # Pass 1, which classifies using the real out_road, not the connector.
                                succ = cr['succ']
                                dest_road = roads[succ['elementId']]
                                dest_at_start = succ.get('contactPoint', 'start') == 'start'
                            else:
                                # Direct link: connecting_road IS the destination real road; it meets
                                # this junction at whichever of its own ends points back at junction_id.
                                dest_s = road_end_touching_junction(cr, junction_id)
                                dest_road = cr
                                dest_at_start = (dest_s is None) or (dest_s == 0.0)
                            out_hdg = classification_heading_away(dest_road, dest_at_start, CLASSIFICATION_LOOKAHEAD)

                            signed_delta = ((out_hdg - in_hdg + math.pi) % (2 * math.pi)) - math.pi
                            bucket = turn_bucket_for_delta(signed_delta)
                            ok = bucket_allowed(tokens, bucket, signed_delta)
                            if not ok:
                                violations.append(
                                    f"way {wid} ({rid}) lane {lane_id} tagged {tokens} but routes to "
                                    f"{connecting_road} lane {to_id} classified as '{bucket}' "
                                    f"(delta={math.degrees(signed_delta):.1f}deg) at junction {junction_id}")
                    if not seen_any:
                        pass  # lane may simply have no destinations (dead end) -- not a violation

    print(f"Checked {checked_ways} tagged way(s) with lane-level turn restrictions "
          f"({total_lane_checks} lane/junction-end checks), {len(violations)} violation(s).")
    if skipped_ways:
        print(f"Skipped {len(skipped_ways)} tagged way(s) not present as a primary road id "
              f"(absorbed into merging/clustering): {', '.join(sorted(skipped_ways))}")
    for v in violations:
        print("  MISMATCH: " + v)
    return len(violations) == 0


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    ok = check(sys.argv[1], sys.argv[2], left_hand='--left-hand-traffic' in sys.argv[3:])
    sys.exit(0 if ok else 1)
