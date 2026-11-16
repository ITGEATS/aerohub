#!/usr/bin/env python3
"""Offline all-COCO diagnostic on saved real RGB inputs, never feeds Tracking."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'src/basic_function/inference/runtime'))
sys.path.insert(0,str(ROOT/'src/basic_function/inference/models/yolo26n_seg'))
from adapter import Predictor, LetterBox, non_max_suppression, scale_boxes
import cv2
import numpy as np
import torch


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    predictor=Predictor();records=[]
    paths=sorted(args.inputs.glob('*.npy'),key=lambda p:int(p.stem))
    # Representative first/middle/last real captures, not synthesized inputs.
    paths=list(dict.fromkeys(paths[i] for i in (0,len(paths)//2,len(paths)-1))) if paths else []
    for path in paths:
        rgb=np.load(path);bgr=np.ascontiguousarray(rgb[:,:,::-1]);h,w=bgr.shape[:2]
        size=predictor.config['input_size']
        letterbox=LetterBox(new_shape=(size,size),auto=False,stride=32)(image=bgr)
        tensor=np.ascontiguousarray(letterbox[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
        outputs,ms=predictor.backend.infer({'image':tensor})
        raw=torch.from_numpy(outputs['raw'])
        boxes=non_max_suppression(raw,conf_thres=predictor.config['confidence'],iou_thres=.7,nc=80,max_det=300)[0]
        boxes[:,:4]=scale_boxes((size,size),boxes[:,:4],(h,w))
        allboxes=[dict(category=predictor.names[int(r[5])],confidence=float(r[4]),xyxy=[float(x) for x in r[:4]]) for r in boxes]
        scores=outputs['raw'][0,4:84].max(axis=1)
        records.append(dict(path=str(path),device=predictor.metadata['device'],
            threshold=predictor.config['confidence'],allclass_boxes=allboxes,
            highest_scores=[dict(category=predictor.names[int(i)],confidence=float(scores[i])) for i in np.argsort(scores)[-10:][::-1]],
            vehicle_class_scores={name:float(scores[index]) for index,name in predictor.names.items() if name in ('car','truck','bus','person')},
            backend_ms=ms))
    args.output.write_text(json.dumps(dict(scope='offline real RGB CUDA allclass diagnosis, no identity claim or tracking input',
        metadata=predictor.metadata,records=records),indent=2)+'\n')

if __name__=='__main__':main()
