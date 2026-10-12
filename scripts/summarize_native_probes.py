#!/usr/bin/env python3
"""Export compact native-policy probe summaries with raw-artifact hashes."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    records = []
    for path in args.inputs:
        raw = path.read_bytes()
        record = json.loads(raw)
        record.pop('samples',None)
        record.update(artifact=path.name,artifact_sha256=hashlib.sha256(raw).hexdigest())
        records.append(record)
    args.output.write_text(json.dumps(records,indent=2)+'\n')
    print(f'Exported {len(records)} probe summaries to {args.output}')


if __name__=='__main__':
    main()
