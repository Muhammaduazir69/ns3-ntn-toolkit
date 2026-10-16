#!/bin/bash
# Runs every campaign group in order; restartable (finished runs are skipped).
cd "$(dirname "$0")"
echo $$ > run_all.pid
for g in hb main eps ablation sens; do
  python3 run_campaign.py $g --jobs ${JOBS:-2} > $g.log 2>&1
done
rm -f run_all.pid
