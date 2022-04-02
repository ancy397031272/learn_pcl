# -*- coding: utf-8 -*-
import numpy as np
import build.example as ex

num = 1000
pcd = [np.random.rand(num), np.random.rand(num), np.random.rand(num), np.random.rand(num), np.random.rand(num),
       np.random.rand(num)]

var = ex.showPCD(pcd)
