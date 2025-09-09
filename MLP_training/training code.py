"""
Neural Network for Control Gain Prediction
------------------------------------------
This network learns a mapping from a target point (x, y)
to optimal control gains (Kp, Kth).

Workflow:
1. Dataset:
   - Input  : Target coordinates (x, y).
   - Output : Control gains (Kp, Kth), found by brute-force search in MATLAB.

2. Model:
   - Feedforward neural network (MLP).
   - Several dense layers with ReLU/tanh activations.
   - Linear output layer (predicts continuous gains).

3. Training:
   - Loss     : Mean Squared Error (MSE).
   - Optimizer: Adam.
   - Callbacks: Early stopping & adaptive learning rate reduction.

4. Evaluation:
   - Reports validation loss.
   - Plots training/validation curves.
   - Optionally shows learning rate evolution.

Author: [Your Name]
"""

# ---------------- Imports ----------------
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, MinMaxScaler

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau


# ---------------- Dataset ----------------
# Load CSV dataset (no header in file)
df = pd.read_csv("full_data_set.csv", header=None)

# Inputs: (x, y), Outputs: (Kp, Kth)
X = df.iloc[:, 0:2].values
y = df.iloc[:, 2:4].values

# Split into training and validation sets
X_train, X_val, y_train, y_val = train_test_split(
    X, y, test_size=0.1, random_state=42
)

X_train_scaled, X_val_scaled = X_train, X_val
y_train_scaled, y_val_scaled = y_train, y_val


# ---------------- Model ----------------
model = Sequential([
    Dense(128, input_dim=2, activation='relu'),
    Dense(64, activation='relu'),
    Dense(64, activation='tanh'),
    Dense(64, activation='relu'),
    Dense(2, activation='linear')  # Outputs continuous values (Kp, Kth)
])

optimizer = Adam(
    learning_rate=0.001,
    beta_1=0.9,
    beta_2=0.999,
    epsilon=1e-07,
    amsgrad=False
)

model.compile(optimizer=optimizer, loss='mse')


# ---------------- Callbacks ----------------
early_stop = EarlyStopping(
    monitor='val_loss',
    patience=50,
    restore_best_weights=True,
    verbose=1
)

reduce_lr = ReduceLROnPlateau(
    monitor='val_loss',
    factor=0.9,    # Reduce learning rate by this factor
    patience=75,   # Epochs without improvement before reducing LR
    min_lr=1e-6,
    verbose=1
)


# ---------------- Training ----------------
history = model.fit(
    X_train_scaled, y_train_scaled,
    validation_data=(X_val_scaled, y_val_scaled),
    epochs=1000,
    batch_size=128,
    callbacks=[reduce_lr, early_stop],
    verbose=1
)


# ---------------- Evaluation ----------------
final_val_loss = model.evaluate(X_val_scaled, y_val_scaled, verbose=0)
print(f"\n✅ Final validation loss: {final_val_loss:.6f}")


# ---------------- Plots ----------------
# Training and validation loss
plt.figure(figsize=(10, 5))
plt.plot(history.history['loss'], label='Training Loss', linewidth=2)
plt.plot(history.history['val_loss'], label='Validation Loss', linewidth=2)
plt.title('Loss Over Epochs')
plt.xlabel('Epoch')
plt.ylabel('MSE Loss')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Learning rate evolution (if logged)
if 'lr' in history.history:
    plt.figure(figsize=(10, 5))
    plt.plot(history.history['lr'], label='Learning Rate')
    plt.title('Learning Rate Over Epochs')
    plt.xlabel('Epoch')
    plt.ylabel('Learning Rate')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


# ---------------- Prediction Example ----------------
# To get predictions:
# y_pred= model.predict(X_val_scaled)


