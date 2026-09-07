#!/usr/bin/env python3
"""Accelerated synthetic numeric timeline; no hardware or camera performance claim."""
import argparse
import json
from pathlib import Path
import resource
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from humanoid_manager.recording.alignment import Aligner,target_time,target_count
from humanoid_manager.recording.buffers import TimeCache
from humanoid_manager.recording.clocks import Clocks
from humanoid_manager.recording.config import validate
from humanoid_manager.recording.models import normalize


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=3600)
    args=parser.parse_args()
    cfg=validate({'sources':{'joints':{'kind':'state','transport':'envelope'}}})
    clocks=Clocks()
    clocks.add({'id':'synthetic','clock_id':'virtual','epoch':0,'device_origin_ns':0,'common_origin_ns':0,
        'uncertainty_ns':0,'evidence':'exact generated timeline, not real-device evidence'})
    cache=TimeCache(cfg['buffers'])
    aligner=Aligner(cfg,cache)
    base=1_000_000_000
    count=target_count(base,base+args.seconds*10**9,30)
    seq=0
    maximum_error=0.
    began=time.monotonic()
    for k in range(count):
        target=target_time(base,k,30)
        deadline=target+80_000_000
        while seq<args.seconds*100 and base+seq*10_000_000<=deadline:
            stamp=base+seq*10_000_000
            d={'source_id':'joints','source_seq':seq,'source_timestamp_ns':stamp,'receive_time_ns':stamp,
                'clock_id':'virtual','clock_epoch':0,'clock_model_id':'synthetic','timestamp_quality':'mapped',
                'payload':{'position':[seq/100.]}}
            cache.add(normalize(d,cfg['sources']['joints'],clocks,'synthetic',stamp),deadline)
            seq+=1
        row=aligner.row(k,base,deadline)
        assert row['valid'],row['invalid_reasons']
        maximum_error=max(maximum_error,abs(row['state']['joints']['position']['value'][0]-(target-base)/1e9))
    assert seq==args.seconds*100
    assert maximum_error<1e-9
    print(json.dumps({'test':'accelerated_synthetic_numeric_only','simulated_seconds':args.seconds,
        'raw_samples_processed':seq,'aligned_rows':count,'max_interpolation_error':maximum_error,
        'cache_peak_bytes':cache.peak_bytes,'cache_final_items':sum(len(v) for v in cache.values.values()),
        'peak_rss_bytes':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,'wall_seconds':round(time.monotonic()-began,3)}))


if __name__=='__main__':
    main()
