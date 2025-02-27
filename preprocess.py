# Script that loads the audio from subfolders, extracts MFCCs, and saves them as NumPy arrays or similar

import os
import librosa
import numpy as np

# Configuration parameters
SAMPLE_RATE = 16000  # Standard sample rate for speech
N_MFCC = 13          # Number of MFCC coefficients to extract
HOP_LENGTH = 512     # Controls the time resolution of MFCCs

def extract_mfcc(file_path, sample_rate=SAMPLE_RATE, n_mfcc=N_MFCC, hop_length=HOP_LENGTH):
    """
    Load an audio file and extract the MFCC features.
    Returns the mean of the MFCC features across time (resulting in a feature vector).
    """
    try:
        # Load the audio file
        signal, sr = librosa.load(file_path, sr=sample_rate)
        # Extract MFCC features
        mfcc = librosa.feature.mfcc(y=signal, sr=sr, n_mfcc=n_mfcc, hop_length=hop_length)
        # Compute the mean of the MFCC coefficients over time to create a fixed-size feature vector
        mfcc_mean = np.mean(mfcc, axis=1)
        return mfcc_mean
    except Exception as e:
        print(f"Error processing {file_path}: {e}")
        return None

def preprocess_data():
    """
    Walk through each subfolder (each representing a command) in the current directory,
    extract MFCC features from each WAV file, and save the resulting features and labels.
    """
    features = []  # List to store MFCC feature vectors
    labels = []    # Corresponding labels

    # Get the current directory (where preprocess.py is located)
    base_dir = os.getcwd()

    # Loop through each item in the current directory
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        # Check if the item is a directory and skip any that are not command folders
        if os.path.isdir(item_path) and not item.startswith('.') and item not in ['__pycache__']:
            command_label = item  # Use the folder name as the command label
            print(f"Processing command: {command_label}")
            # Process each WAV file in the command folder
            for file_name in os.listdir(item_path):
                if file_name.lower().endswith(".wav"):
                    file_path = os.path.join(item_path, file_name)
                    mfcc_features = extract_mfcc(file_path)
                    if mfcc_features is not None:
                        features.append(mfcc_features)
                        labels.append(command_label)
    
    # Convert lists to NumPy arrays
    X = np.array(features)
    y = np.array(labels)

    print(f"Extracted features shape: {X.shape}")
    print(f"Labels shape: {y.shape}")

    # Save the features and labels to disk for later use
    np.save("features.npy", X)
    np.save("labels.npy", y)
    print("Features and labels saved as 'features.npy' and 'labels.npy'.")

if __name__ == "__main__":
    preprocess_data()
