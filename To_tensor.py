import os
import csv
import torch

"""
This script takes a csv file and save it as a tensor file
"""

data_directory = '/home/naor/Desktop/naor/ai_gps/data/'

# Initialize lists to store data
all_data = []

# Iterate over all CSV files in the directory
for filename in os.listdir(data_directory):
    if filename.endswith(".csv"):
        csv_file_path = os.path.join(data_directory, filename)

        # Open the CSV file and read data
        with open(csv_file_path, mode='r') as file:
            csv_reader = csv.reader(file)
            # Skip the header
            next(csv_reader)
            # Read each row and convert it to a PyTorch tensor
            data = []
            for row in csv_reader:
                # Convert each value in the row to float and create a tensor
                tensor_row = torch.tensor([float(value) for value in row])
                # Append the tensor to the data list
                data.append(tensor_row)

        # Convert the list of tensors to a single tensor and append to all_data
        all_data.append(torch.stack(data))

# Concatenate all tensors in all_data along the first dimension to create a single tensor
full_data = torch.cat(all_data, dim=0)

# Save the tensor to a file in the data directory
torch.save(full_data, os.path.join(data_directory, 'full_data.pt'))

# Print the shape of the full tensor
print("Shape of full tensor data:", full_data.shape)
