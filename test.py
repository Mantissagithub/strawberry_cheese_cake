import torch
import sys

def cuda_diagnostic():
    print(f"Python Version: {sys.version}")
    print(f"CUDA Available: {torch.cuda.is_available()}")
    print(f"CUDA Device Count: {torch.cuda.device_count()}")
    try:
        print(f"CUDA Device Name: {torch.cuda.get_device_name(0)}")
    except Exception as e:
        print(f"Device Name Error: {e}")

cuda_diagnostic()
