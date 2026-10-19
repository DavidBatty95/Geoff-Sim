"""Export explicitly synthetic actor-weight interpolations with endpoint provenance."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left',required=True,type=Path)
    parser.add_argument('--right',required=True,type=Path)
    parser.add_argument('--right-weight',required=True,type=float)
    parser.add_argument('--layer-weights',type=float,nargs=4,help='Override weights for the four linear actor layers')
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.right_weight) or not 0.<args.right_weight<1.:
        parser.error('interior finite weight required')
    weights = args.layer_weights or [args.right_weight]*4
    if not all(math.isfinite(v) and 0.<=v<=1. for v in weights):
        parser.error('finite per-layer weights in [0,1] required')
    left,right = [torch.load(path,map_location='cpu',weights_only=False) for path in (args.left,args.right)]
    a,b = left['actor_state_dict'],right['actor_state_dict']
    if a.keys()!=b.keys() or any(a[k].shape!=b[k].shape for k in a):
        raise ValueError('incompatible endpoint actor states')
    if any(k.startswith('obs_normalizer.') for k in a):
        raise ValueError('normalized actor not supported')
    for key in a:
        if key.startswith('mlp.'):
            weight = weights[int(key.split('.')[1])//2]
            a[key] = (1.-weight)*a[key]+weight*b[key]
            if not torch.isfinite(a[key]).all():
                raise ValueError('nonfinite interpolation')
    args.output.mkdir(parents=True,exist_ok=False)
    # This actor-only checkpoint is for export, not a resumable training run.
    checkpoint = args.output/'interpolated_actor.pt'
    torch.save(dict(actor_state_dict=a,iter=-1),checkpoint)
    manifest = dict(mode='actor_weight_interpolation',not_a_training_run=True,
        left=dict(path=str(args.left),sha256=hashlib.sha256(args.left.read_bytes()).hexdigest()),
        right=dict(path=str(args.right),sha256=hashlib.sha256(args.right.read_bytes()).hexdigest()),
        right_weight=args.right_weight,layer_weights=weights,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (args.output/'interpolation_source.py').write_bytes(Path(__file__).read_bytes())
    subprocess.run([sys.executable,str(Path(__file__).with_name('export_payload_actor.py')),
                    str(checkpoint),'--output',str(args.output/'policy.onnx')],check=True)


if __name__=='__main__':
    main()
