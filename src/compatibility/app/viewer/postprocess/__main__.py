"""Run a local, reviewable postprocessing job configuration."""
import argparse
from .job import run_job
from .lifecycle import read_json

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('configuration')
arguments = parser.parse_args()
run_job(read_json(arguments.configuration))
