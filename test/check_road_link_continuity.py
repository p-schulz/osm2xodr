#!/usr/bin/env python3
"""Verify plain (non-junction) road-to-road lane link continuity in an osm2xodr-generated .xodr.

check_junction_continuity.py already verifies position/heading continuity for every
<junction><connection><laneLink>. It does not check the *other* kind of link OpenDRIVE roads can
carry: a road whose own top-level <predecessor>/<successor> points directly at elementType="road"
(no junction in between) -- e.g. two roads meeting at a feature-split node (a traffic light /
stop sign) where road merging deliberately stops, or any other plain road-to-road boundary. This
script closes that gap: for every road's first/last <laneSection>, it reads each lane's own
<link><predecessor/successor id="..."/></link>, and cross-checks that the neighbor road actually
has a matching lane at the matching contact point, with matching position and heading, and that
the link is symmetric (the neighbor links back).

Usage:
    python3 check_road_link_continuity.py <file.xodr> [--left-hand-traffic]

Exit code is 0 if every checked lane link matches within 1mm / 0.001rad and is symmetric, 1
otherwise.
"""
import math
import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_junction_continuity import parse_xodr, lane_centerline_at_s, angle_diff


def parse_lane_links(body):
    """Returns (first_section_links, last_section_links): {lane_id: {'predecessor': id|None, 'successor': id|None}}
    for the lanes of the first and last <laneSection> of a road body (may be the same section)."""
    sections = list(re.finditer(r'<laneSection\b[^>]*>(.*?)</laneSection>', body, re.S))
    if not sections:
        return {}, {}

    def extract(sec_body):
        links = {}
        for side_m in re.finditer(r'<(left|right)>(.*?)</\1>', sec_body, re.S):
            for lane_m in re.finditer(r'<lane\s+id="(-?\d+)"[^>]*>(.*?)</lane>', side_m.group(2), re.S):
                lid = int(lane_m.group(1))
                pred_m = re.search(r'<predecessor\s+id="(-?\d+)"', lane_m.group(2))
                succ_m = re.search(r'<successor\s+id="(-?\d+)"', lane_m.group(2))
                links[lid] = {
                    'predecessor': int(pred_m.group(1)) if pred_m else None,
                    'successor': int(succ_m.group(1)) if succ_m else None,
                }
        return links

    return extract(sections[0].group(1)), extract(sections[-1].group(1))


def contact_s(road, contact_point):
    return 0.0 if contact_point == 'start' else road['length']


def check(path, left_hand):
    data = open(path, encoding='utf-8').read()
    roads = parse_xodr(data)
    lane_links = {rid: parse_lane_links(r['body']) for rid, r in roads.items()}

    total = 0
    bad = 0
    checked_pairs = set()

    def check_direction(rid, link_elem, contact_end_s, section_links, direction):
        # `direction` is which sub-key of each lane's <link> actually points at this neighbor:
        # 'predecessor' when called for the road's own <predecessor>, 'successor' for its own
        # <successor>. The *other* sub-key of the same lane (if present) points at an internal
        # laneSection boundary within this same road, not at the neighbor -- checking it here
        # would compare an unrelated internal link against this external neighbor.
        nonlocal total, bad
        if not link_elem or link_elem.get('elementType') != 'road':
            return
        neighbor_id = link_elem['elementId']
        if neighbor_id not in roads:
            print(f"  {rid}: links to unknown road '{neighbor_id}' [MISMATCH]")
            bad += 1
            total += 1
            return
        neighbor = roads[neighbor_id]
        contact_point = link_elem.get('contactPoint', 'start')
        neighbor_s = contact_s(neighbor, contact_point)
        neighbor_first, neighbor_last = lane_links[neighbor_id]
        neighbor_section_links = neighbor_first if contact_point == 'start' else neighbor_last
        neighbor_direction = 'successor' if direction == 'predecessor' else 'predecessor'

        pair_key = tuple(sorted([(rid, contact_end_s, direction), (neighbor_id, neighbor_s, neighbor_direction)]))
        if pair_key in checked_pairs:
            return
        checked_pairs.add(pair_key)

        for lane_id, link in section_links.items():
            target_lane = link.get(direction)
            if target_lane is None:
                continue
            total += 1
            x1, y1, h1 = lane_centerline_at_s(roads, rid, lane_id, contact_end_s, left_hand)
            if target_lane not in neighbor_section_links:
                print(f"  {rid} lane {lane_id} {direction}->{neighbor_id} lane {target_lane}: "
                      f"neighbor has no such lane at that end [MISMATCH]")
                bad += 1
                continue
            x2, y2, h2 = lane_centerline_at_s(roads, neighbor_id, target_lane, neighbor_s, left_hand)
            dpos = math.hypot(x1 - x2, y1 - y2)
            dhdg = angle_diff(h1, h2)
            back = neighbor_section_links[target_lane].get(neighbor_direction)
            symmetric = (back == lane_id)
            ok = dpos < 1e-3 and dhdg < 1e-3 and symmetric
            if not ok:
                bad += 1
                print(f"  {rid} lane {lane_id} {direction} -> {neighbor_id} lane {target_lane}: "
                      f"dpos={dpos:.5f}m dhdg={math.degrees(dhdg):.3f}deg "
                      f"symmetric={symmetric} [MISMATCH]")

    for rid, road in roads.items():
        if road['attrs'].get('junction', '-1') != '-1':
            continue  # junction connector roads are covered by check_junction_continuity.py
        first_links, last_links = lane_links[rid]
        check_direction(rid, road['pred'], 0.0, first_links, 'predecessor')
        check_direction(rid, road['succ'], road['length'], last_links, 'successor')

    print(f"Checked {total} plain road-to-road lane link(s), {bad} mismatched (> 1mm / 0.001rad, or asymmetric).")
    return bad == 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    ok = check(sys.argv[1], left_hand='--left-hand-traffic' in sys.argv[2:])
    sys.exit(0 if ok else 1)
