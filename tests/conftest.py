import os
import sys

# Make the synthetic-log helper importable as `synthlog` from every test module.
sys.path.insert(0, os.path.dirname(__file__))
