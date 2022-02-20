# -*- coding: utf-8 -*-
"""
Created on Mon Aug 30 21:12:31 2021

@author: DELL
"""

# 引用Paddlelite预测库
from paddlelite.lite import *

# 1. 创建opt实例
opt=Opt()
# 2. 指定输入模型地址 
opt.set_model_file("./models")
opt.set_param_file("./params")
# 3. 指定转化类型： arm、x86、opencl、npu
opt.set_valid_places("arm")
# 4. 指定模型转化类型： naive_buffer、protobuf
opt.set_model_type("naive_buffer")
# 4. 输出模型地址
opt.set_optimize_out("./mobilenetv1_opt")
# 5. 执行模型优化
opt.run()