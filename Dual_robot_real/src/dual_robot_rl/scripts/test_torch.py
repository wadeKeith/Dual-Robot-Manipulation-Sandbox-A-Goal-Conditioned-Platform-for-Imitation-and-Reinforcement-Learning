import torch
 
# 检查CUDA是否可用
cuda_available = torch.cuda.is_available()
print("CUDA available:", cuda_available)
 
# 如果CUDA可用，打印CUDA设备数量
if cuda_available:
    device_count = torch.cuda.device_count()
    print("Number of CUDA devices:", device_count)
 
    # 打印每个CUDA设备的名称和内存
    for i in range(device_count):
        print("Device {}: {} ({} MB memory)".format(i,
                                                   torch.cuda.get_device_name(i),
                                                   torch.cuda.get_device_properties(i).total_memory / 1024 ** 2))
