#!/usr/bin/env python3
"""Unknown-layout cube search, pickup, destination search and delivery demo."""
import sys

from run_transport_demo import main


if __name__=='__main__':
    sys.argv[1:1] = ['--world','transport_medium','--explore-unknown',
                    '--people','1','--duration','600','--search-max-viewpoints','40','--carry-posture','payload_v1']
    raise SystemExit(main())
