import torch
import torch.nn as nn
import joblib
import numpy as np

# Load the scaler
scaler = joblib.load('delta_estimator_mlp/scaler.pkl')

# Define the model
class DeltaEstimatorMLP(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(DeltaEstimatorMLP, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size, output_size)
        self.tanh = nn.Tanh()

    def forward(self, x):
        out = self.fc1(x)
        out = self.relu(out)
        out = self.fc2(out)
        out = self.tanh(out)
        return out

# Parameters
input_size = 9  # Number of input features
hidden_size = 32  # Number of neurons in the hidden layer
output_size = 2  # Number of output features (delta_y, delta_z)

# Create model and load trained weights
model = DeltaEstimatorMLP(input_size, hidden_size, output_size)
model.load_state_dict(torch.load('delta_estimator_mlp/delta_estimator_mlp.pth'))
model.eval()

# Sample inference code
def infer(input_features):
    # Convert input features to tensor
    input_tensor = torch.tensor(input_features, dtype=torch.float32).unsqueeze(0)  # Add batch dimension
    
    # Perform inference
    with torch.no_grad():
        prediction = model(input_tensor)
    
    # Inverse transform the prediction to get original values
    prediction_original = scaler.inverse_transform(prediction.numpy())
    
    return prediction_original[0]

# Example input features for inference
sample_input = np.array([
    0.5, 0.5, 0.5,  # ball_initial_pos_x, ball_initial_pos_y, ball_initial_pos_z
    1.0,  # distance_ball_robot
    0.1, 0.1, 0.1,  # estimated_initial_velocity_x, estimated_initial_velocity_y, estimated_initial_velocity_z
    0.2, 0.2  # estimated_hitting_point_y, estimated_hitting_point_z
])

# Perform inference and print the result
predicted_output = infer(sample_input)
print("Predicted Output:", predicted_output)

