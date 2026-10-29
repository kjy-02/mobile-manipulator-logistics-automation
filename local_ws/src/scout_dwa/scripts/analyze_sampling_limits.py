#!/usr/bin/env python3
"""Replay pinned upstream sampling arithmetic; a missed point is a limitation."""
import argparse
import json
import math
import struct
import yaml

def f32(v):
    return struct.unpack('f', struct.pack('f', v))[0]

def analyze(dwa, adapter):
    increment=f32(adapter['scan_angle_increment'])
    step=int(dwa['ANGLE_RESOLUTION']/increment)
    if step<1:
        raise ValueError('zero upstream scan step')
    amin=f32(adapter['scan_angle_min']); amax=f32(adapter['scan_angle_max'])
    bins=int(math.floor((amax-amin)/increment))+1
    scan_missed=[i for i in range(bins) if i % step]
    resolution=f32(adapter['map_resolution'])
    width=math.ceil(adapter['map_size_x']/adapter['map_resolution'])
    height=math.ceil(adapter['map_size_y']/adapter['map_resolution'])
    ox=-.5*width*resolution; oy=-.5*height*resolution
    sampled=set()
    angle=f32(-math.pi)
    max_distance=math.hypot(ox,oy)
    while angle<=math.pi:
        distance=f32(0)
        while distance<=max_distance:
            ix=math.floor((distance*math.cos(angle)-ox)/resolution)
            iy=math.floor((distance*math.sin(angle)-oy)/resolution)
            if 0<=ix<width and 0<=iy<height:
                sampled.add((ix,iy))
            distance=f32(distance+resolution)
        angle=f32(angle+dwa['ANGLE_RESOLUTION'])
    range_test=2.0
    map_missed=[]
    for degree in range(-180,180):
        rad=math.radians(degree)
        ix=math.floor((range_test*math.cos(rad)-ox)/resolution)
        iy=math.floor((range_test*math.sin(rad)-oy)/resolution)
        if 0<=ix<width and 0<=iy<height and (ix,iy) not in sampled:
            map_missed.append(degree)
    return dict(
        analysis='sampling arithmetic replay; not a live planner collision test',
        upstream_commit='6387a9660c2064fa0545ac81d55f4bbe7962651f',
        scan_float32_increment=increment, upstream_scan_step=step,
        scan_bins=bins, scan_single_bin_missed_count=len(scan_missed),
        scan_single_bin_missed_examples=scan_missed[:10],
        map_test_range_m=range_test, map_test_angles=360,
        map_single_cell_missed_count=len(map_missed),
        map_single_cell_missed_angles_examples=map_missed[:20],
        candidate_scan_spread_bins=step-1,
        candidate_max_angle_displacement_rad=(step-1)*increment,
        candidate_max_displacement_at_2m=2*range_test*math.sin((step-1)*increment/2),
        candidate_status='not enabled; scan spreading needs geometric tests; map limitation remains',
        motion_readiness='not established by this analysis')

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('dwa_yaml'); parser.add_argument('adapter_yaml')
    args=parser.parse_args()
    with open(args.dwa_yaml,encoding='utf-8') as s: dwa=yaml.safe_load(s)
    with open(args.adapter_yaml,encoding='utf-8') as s: adapter=yaml.safe_load(s)
    print(json.dumps(analyze(dwa,adapter),ensure_ascii=False,indent=2))
