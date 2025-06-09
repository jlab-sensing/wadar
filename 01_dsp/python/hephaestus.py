# Hephaestus (Ἥφαιστος): God of blacksmiths, metalworking, and craftsmanship. This file sses functions for shaping, forging, and 
# transforming data into features.

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from hydros import HydrosFrameLoader
from eos import EosDenoising
import os
from scipy import signal

dataset_dir = "data/compact-4-dry"
hydros = HydrosFrameLoader(dataset_dir)
X, y = hydros.load_from_dataset()

test_frame = X[0]
test_frame = np.median(test_frame, axis=1)

plt.figure()
plt.plot(np.abs(test_frame), label='Original Signal', alpha=0.5)
plt.show()