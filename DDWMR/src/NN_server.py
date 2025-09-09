#!/usr/bin/env python3
"""
Neural Network Gains Service Node

This ROS service node takes target (x, y) as input and outputs
the predicted optimal control gains (Kp, Kth) using a trained
MLP network stored in 'MLP2.h5'.

Workflow:
1. Load network weights and biases from HDF5.
2. Convert them to PyTorch tensors.
3. On service request:
   - Forward propagate input (x, y) through the network using ReLU/tanh activations.
   - Extract network output and map to control gains.
4. Return Kp, Kth via `gains.srv`.
"""

import rospy
from control.srv import gains, gainsResponse
import h5py
import numpy as np
import torch
import torch.nn.functional as F

# ==============================
# ROS Service Callback
# ==============================
def compute_gains(request): 
    """
    Compute Kp, Kth control gains using the trained MLP network.
    """
    # Prepare input as PyTorch tensor
    x_net = request.x
    y_net = request.y
    Input = torch.tensor([[x_net], [y_net]], dtype=torch.float32)

    # Forward pass through MLP with ReLU/tanh activations
    L1 = F.relu(W0_1 @ Input + B1)
    L2 = F.relu(W1_2 @ L1 + B2)
    L3 = torch.tanh(W2_3 @ L2 + B3)
    L4 = F.relu(W3_4 @ L3 + B4)
    output = W4_5 @ L4 + B5

    # Extract outputs and map to control gains
    Kp = output[0, 0].item()
    Kth = output[1, 0].item()

    # Optional mapping / clamping
    Kp_mapped = min(0.2, Kp - 0.3)
    Kth_mapped = Kth + 1

    return gainsResponse(Kp_mapped, Kth_mapped)

# ==============================
# Node Initialization
# ==============================
rospy.init_node("NN")
rospy.loginfo("🧠 Neural Network Gains Service started")

# ==============================
# Load MLP weights from HDF5
# ==============================
with h5py.File('/home/pi/test_ws/src/control/src/MLP2.h5', 'r') as file:
    # Layer 0 -> 1
    W0_1 = np.array(file['model_weights/dense/sequential/dense/kernel']).T
    B1   = np.array([file['model_weights/dense/sequential/dense/bias']]).T

    # Layer 1 -> 2
    W1_2 = np.array(file['/model_weights/dense_1/sequential/dense_1/kernel']).T
    B2   = np.array([file['/model_weights/dense_1/sequential/dense_1/bias']]).T

    # Layer 2 -> 3
    W2_3 = np.array(file['/model_weights/dense_2/sequential/dense_2/kernel']).T
    B3   = np.array([file['/model_weights/dense_2/sequential/dense_2/bias']]).T

    # Layer 3 -> 4
    W3_4 = np.array(file['/model_weights/dense_3/sequential/dense_3/kernel']).T
    B4   = np.array([file['/model_weights/dense_3/sequential/dense_3/bias']]).T

    # Layer 4 -> 5
    W4_5 = np.array(file['/model_weights/dense_4/sequential/dense_4/kernel']).T
    B5   = np.array([file['/model_weights/dense_4/sequential/dense_4/bias']]).T

# Convert all weights and biases to PyTorch tensors
W0_1 = torch.tensor(W0_1, dtype=torch.float32)
B1   = torch.tensor(B1, dtype=torch.float32)
W1_2 = torch.tensor(W1_2, dtype=torch.float32)
B2   = torch.tensor(B2, dtype=torch.float32)
W2_3 = torch.tensor(W2_3, dtype=torch.float32)
B3   = torch.tensor(B3, dtype=torch.float32)
W3_4 = torch.tensor(W3_4, dtype=torch.float32)
B4   = torch.tensor(B4, dtype=torch.float32)
W4_5 = torch.tensor(W4_5, dtype=torch.float32)
B5   = torch.tensor(B5, dtype=torch.float32)

# ==============================
# Advertise ROS Service
# ==============================
service = rospy.Service("gains", gains, compute_gains)
rospy.loginfo("✅ Gains service is ready")
rospy.spin()

