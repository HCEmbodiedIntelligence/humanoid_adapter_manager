#!/usr/bin/env python3
"""Validate episode endpoints using the pinned upstream LeRobot reader, offline."""
import argparse
import importlib.metadata
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path)
    args=parser.parse_args()
    if importlib.metadata.version('lerobot')!='0.4.4':
        parser.error('此验收脚本需要 LeRobot 0.4.4；请在独立训练环境安装固定版本')
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset=LeRobotDataset('local/session_validation',root=args.dataset.resolve(),video_backend='pyav')
    info=json.loads((args.dataset/'meta/info.json').read_text())
    checked=[]
    for ep in dataset.meta.episodes:
        for index in {ep['dataset_from_index'],ep['dataset_to_index']-1}:
            row=dataset[index]
            local=index-ep['dataset_from_index']
            assert int(row['frame_index'])==local
            assert int(row['episode_index'])==ep['episode_index']
            assert abs(float(row['timestamp'])-local/info['fps'])<1e-4
            for key,feature in info['features'].items():
                if feature['dtype']=='video':
                    height,width,channels=feature['shape']
                    assert tuple(row[key].shape)==(channels,height,width)
            checked.append({'index':index,'episode_index':ep['episode_index'],'frame_index':local})
    print(json.dumps({'lerobot':'0.4.4','frames':len(dataset),'episodes':len(dataset.meta.episodes),'checked_endpoints':checked},ensure_ascii=False))


if __name__=='__main__':
    main()
