#!/bin/bash

for i in {0..49}; do
    sbatch \
        --job-name=GetResults_${i} \
        --output=cpu_report/r-cpu-test-${i}-%j.out \
        --error=cpu_report/r-cpu-test-${i}-%j.err \
        slurm.sh $i
done