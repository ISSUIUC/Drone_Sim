import numpy as np

def forward_euler(f, t_s, x, h_s):
    for i in range(1, len(t_s)):
        x[:, i] = x[:, i-1] + h_s * f(t_s[i-1], x[:,i-1]) # forward euler formula
    return t_s, x