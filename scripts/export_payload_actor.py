#!/usr/bin/env python3
"""Export a local payload-training checkpoint with strict layout and parity checks.

Run in the MJLab uv environment. Output and provenance are never overwritten.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from onnx.reference import ReferenceEvaluator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkpoint',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    torch.set_num_threads(2)
    metadata = args.output.with_suffix('.manifest.json')
    if args.output.exists() or metadata.exists():
        parser.error('output exists; retain every exported candidate separately')
    checkpoint = torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    state = checkpoint['actor_state_dict']
    if any(key.startswith('obs_normalizer.') for key in state):
        raise ValueError('this exporter requires the unnormalized deployed actor contract')
    actor = torch.nn.Sequential(torch.nn.Linear(480,512),torch.nn.ELU(),
        torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,29)).eval()
    actor.load_state_dict({key.removeprefix('mlp.'):value for key,value in state.items() if key.startswith('mlp.')},strict=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    torch.onnx.export(actor,torch.zeros(1,480),str(args.output),input_names=['obs'],output_names=['actions'],
                      opset_version=17,dynamo=False)
    rng = np.random.default_rng(75)
    errors = []
    evaluator = ReferenceEvaluator(str(args.output))
    for _ in range(8):
        sample = rng.normal(0.,.2,(1,480)).astype(np.float32)
        with torch.no_grad():
            expected = actor(torch.from_numpy(sample)).numpy()
        actual = evaluator.run(None,{'obs':sample})[0]
        np.testing.assert_allclose(actual,expected,atol=2e-5,rtol=2e-5)
        errors.append(float(np.max(np.abs(actual-expected))))
    report = dict(checkpoint=str(args.checkpoint),iteration=checkpoint['iter'],
                  checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                  policy_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                  parity_max_error=max(errors),actor_contract='480/29, unnormalized, term-major five-frame history')
    metadata.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    main()
